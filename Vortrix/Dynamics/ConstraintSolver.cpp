#include "ConstraintSolver.h"

#include "Core/ScratchAllocator.h"

namespace vx {

	void ConstraintSolver::Init(const BodyManager& body_manager)
	{
		uint32 max_bodies = body_manager.MaxBodies();
		mBodies.reserve(max_bodies);
		mBodyToSolverBody.resize(max_bodies);
		mConstraintPositionSolveQueue.reserve(100);

		//Frame init for linear 1D row with scrachAllocator
	}

	ConstraintSolver::~ConstraintSolver()
	{
		mBodies.clear();
		mBodyToSolverBody.clear();


		//the remove the below in order 
		mConstraintPositionSolveQueue.clear();
		//mLinear1DRows.clear();
	}

	SolverBodyIndex ConstraintSolver::GetOrCreateSolverBody(BodyID physics_body_id, const PhysicsStepContext& ctx)
	{
		SolverBodyIndex& solver_idx = mBodyToSolverBody[physics_body_id.Idx()];

		//if (solver_idx.Value() >= 0)
		if (solver_idx.IsValid())
			return solver_idx;


		const Body& body = ctx.bodyManager->GetBody(physics_body_id);

		SolverBody solver_body;
		solver_body.bodyID = physics_body_id;

		if (body.IsStatic())
		{
			solver_body.v = Vec3::Zero();
			solver_body.w = Vec3::Zero();
			solver_body.invMass = 0.0f;
		}
		else
		{
			solver_body.v = body.GetLinearVelocity();
			solver_body.w = body.GetAngularVelocity();
			solver_body.invMass = body.GetInverseMass();
		}

		uint32 new_idx = uint32(mBodies.size());
		mBodies.push_back(solver_body);

		solver_idx = SolverBodyIndex(new_idx);
		//mBodyToSolverBody[physics_body_id.Value()] = new_idx;

		return solver_idx;
	}

	SolverBodyIndex ConstraintSolver::GetOrCreateSolverBody(const Body& body)
	{
		SolverBodyIndex& solver_idx = mBodyToSolverBody[body.GetID().Idx()];

		//if (solver_idx.Value() >= 0)
		if (solver_idx.IsValid())
			return solver_idx;


		SolverBody solver_body;
		solver_body.bodyID = body.GetID();

		if (body.IsStatic())
		{
			solver_body.v = Vec3::Zero();
			solver_body.w = Vec3::Zero();
			solver_body.invMass = 0.0f;
		}
		else
		{
			solver_body.v = body.GetLinearVelocity();
			solver_body.w = body.GetAngularVelocity();
			solver_body.invMass = body.GetInverseMass();
		}

		uint32 new_idx = uint32(mBodies.size());
		mBodies.push_back(solver_body);

		solver_idx = SolverBodyIndex(new_idx);
		//mBodyToSolverBody[physics_body_id.Value()] = new_idx;

		return solver_idx;
	}

	void ConstraintSolver::WriteBackBodies(const SolverBody* bodies, uint32 count, BodyManager& body_manager)
	{
		VX_PROFILE_FUNCTION();
		for (const SolverBody* sb = bodies, *sb_end = bodies + count; sb < sb_end; ++sb)
		{
			if ((*sb).invMass == 0)
				continue;
			WriteBackBody(*sb, body_manager.GetBody((*sb).bodyID));
		}
	}

	void ConstraintSolver::PrepareSolver(uint32 required_liner_row, const PhysicsStepContext& ctx)
	{
		mLinear1DRowBufferCount = required_liner_row;
		mLinear1DRows = reinterpret_cast<Linear1DRow*>(ctx.mScratchAllocator->Allocate(sizeof(Linear1DRow) * required_liner_row));
		//ensure mem is clean and defualt to Linear1DROw
		std::memset(mLinear1DRows, {}, mLinear1DRowBufferCount * sizeof(Linear1DRow));
	}

	void ConstraintSolver::HackClear()
	{
		//mLinear1DRows.clear();

		mLinear1DRowsCounts = 0;
		mLinear1DRowBufferCount = 0;

		//quick hack, no caching pipeline and to prevent bugs 
		mBodies.clear();
		std::fill(mBodyToSolverBody.begin(), mBodyToSolverBody.end(), SolverBodyIndex{});
		mConstraintPositionSolveQueue.clear();
	}

	void ConstraintSolver::SolverVelocityLinear1DRow(Linear1DRow& row, SolverBody* bodies)
	{
		VX_PROFILE_FUNCTION();
		SolverBody& sbA = bodies[row.bodyAidx.Value()];
		SolverBody& sbB = bodies[row.bodyBidx.Value()];

		Vec3 axis = Vec3::LoadFloat3Raw(row.axis);

		//jacobian 
		float jv = axis.Dot(sbA.v - sbB.v) +
			Vec3::LoadFloat3Raw(row.rAXn).Dot(sbA.w) -
			Vec3::LoadFloat3Raw(row.rBXn).Dot(sbB.w);

		///according to jolt's total bias inclind supplied bias 
		/// jv + (beta/h)*C + (gamma * lamba)
		/// jv + bias + gamma * lamda
		float compliance = row.gamma * row.lambda + row.bias;

		float lambda = (jv - compliance) * row.effMass;
		//float lambda = -(jv + compliance) * row.effMass;

		float _lambda = VxClamp(row.lambda + lambda, row.minLambda, row.maxLambda);
		float impluse = _lambda - row.lambda;
		row.lambda = _lambda;

		//store changes
		sbA.v -= impluse * sbA.invMass * axis;
		sbA.w -= impluse * Vec3::LoadFloat3Raw(row.invIrAXn);
		sbB.v += impluse * sbB.invMass * axis;
		sbB.w += impluse * Vec3::LoadFloat3Raw(row.invIrBXn);

		VX_ASSERT(!sbA.v.IsNaN(), "lin_velA is nan");
		VX_ASSERT(!sbA.w.IsNaN(), "ang_velA is nan");
		VX_ASSERT(!sbB.v.IsNaN(), "lin_velB is nan");
		VX_ASSERT(!sbB.w.IsNaN(), "ang_velB is nan");
	}

	void ConstraintSolver::SolverVelocityLinear1DRows(Linear1DRow* rows, size_t begin_offset, size_t count, SolverBody* bodies)
	{
		VX_PROFILE_FUNCTION();
		//for (Linear1DRow** r = rows, **r_end = rows + count; r < r_end; ++r)
		for (Linear1DRow* r = rows, *r_end = rows + count; r < r_end; ++r)
			SolverVelocityLinear1DRow(*r, bodies);
	}

	void ConstraintSolver::SolveConstraintsPosition(Constraint** constraints, size_t count, float dt, float baumgarte)
	{
		VX_PROFILE_FUNCTION();

		for (Constraint** c = constraints, **c_end = constraints + count; c < c_end; ++c)
			(*c)->SolvePositionConstraint(dt, baumgarte);
	}

	VX_INLINE void ConstraintSolver::WarmStart(const Linear1DRow& row, SolverBody& body0, SolverBody& body1)
	{
		if (row.lambda == 0.0f)
			return;

		//row.lambda *= 0.8f;
		float impluse = row.lambda;

		Vec3 axis = Vec3::LoadFloat3Raw(row.axis);

		body0.v -= impluse * body0.invMass * axis;
		body0.w -= impluse * Vec3::LoadFloat3Raw(row.invIrAXn);

		body1.v += impluse * body1.invMass * axis;
		body1.w += impluse * Vec3::LoadFloat3Raw(row.invIrBXn);
	}

	void ConstraintSolver::WarmStart(Linear1DRow* rows, size_t begin_offset, size_t count, SolverBody* bodies)
	{
		VX_PROFILE_FUNCTION();
		for (Linear1DRow* r = rows, *r_end = rows + count; r < r_end; ++r)
		{
			Linear1DRow& row = (*r);
			SolverBody& sbA = bodies[row.bodyAidx.Value()];
			SolverBody& sbB = bodies[row.bodyBidx.Value()];

			WarmStart(row, sbA, sbB);
		}
	}

	void ConstraintSolver::SolverAll(const PhysicsStepContext& ctx, uint32 iterations)
	{
		VX_PROFILE_FUNCTION();
		//ConstraintSolver::WarmStart(mLinear1DRows.data(), 0, mLinear1DRows.size(), mBodies.data());

		//for (int i = 0; i < iterations; ++i)
		//	ConstraintSolver::SolverVelocityLinear1DRows(mLinear1DRows.data(), 0, mLinear1DRows.size(), mBodies.data());
		//{
		//	VX_PROFILE_SCOPE("ConstraintSolver Solve all commit state");
		//	for (const auto& r : mLinear1DRows)
		//		if (r.user)
		//			r.user->CommitSolverState(r);
		//}

		ConstraintSolver::WarmStart(mLinear1DRows, 0, mLinear1DRowsCounts, mBodies.data());

		for (int i = 0; i < iterations; ++i)
			ConstraintSolver::SolverVelocityLinear1DRows(mLinear1DRows, 0, mLinear1DRowsCounts, mBodies.data());

			CommitStateConstraint();

		ConstraintSolver::WriteBackBodies(mBodies.data(), mBodies.size(), *ctx.bodyManager);
	}

} //namespace vx
