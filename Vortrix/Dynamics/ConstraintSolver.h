#pragma once

#include "Maths/Vec3.h"
#include "Body/BodyID.h"
#include "SolverBodyIndex.h"

#include "Body/BodyManager.h"

#include "Constraints/Constraint.h"

#include "Core/Profiler.h"

#include "ConstraintSolverRow.h"

namespace vx {



	//struct Island
	//{
	//	SolverBody* mBodies
	//};


	/// later have a solver builder (which includes island bases) 
	class ConstraintSolver
	{
	private:
		/// later make thread safe, in the term of islands/graph
		/// would have portions of bodies they can touch,
		/// or even sort plus segment, that either
		/// thread0 get a list to work on based on graph
		/// and thread1 get a different list 
		/// 
		/// or segmented buffer with cache line alignment 
		/// that cache line does not overlap that 
		/// thread0 could work on bodies of first few cache line 
		/// while thread1 works on next few cache line bodies etc 
		std::vector <SolverBody> mBodies;
		std::vector<SolverBodyIndex> mBodyToSolverBody;
		std::vector<Constraint*> mConstraintPositionSolveQueue;

	public:

		void Init(const BodyManager& body_manager)
		{
			uint32 max_bodies = body_manager.MaxBodies();
			mBodies.reserve(max_bodies);
			mBodyToSolverBody.resize(max_bodies);
			mConstraintPositionSolveQueue.reserve(100);
		}

		~ConstraintSolver()
		{
			mBodies.clear();
			mBodyToSolverBody.clear();
			mConstraintPositionSolveQueue.clear();
			mLinear1DRows.clear();
		}

		///in order to trck is bodies is already participamt with another constraint 
		///because 
		/// a single body should not have multiple instance of solver body
		/// for instance: 
		///		Constraint0 -> connected to BodyA and B
		///		constraint1 -> connected to Body B & C
		/// Constraint0 & Constraint1; their BodyB should have the same Solver body. 
		/// So has to solving Constraint1 its influence on BodyB have to propagated to 
		/// continue influence from Constraint0
		/// 
		/// might also introduce Solver Body have its own id like a feature id
		/// that can be shared with other constraints 
		SolverBodyIndex GetOrCreateSolverBody(BodyID physics_body_id, const PhysicsStepContext& ctx)
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

		SolverBodyIndex GetOrCreateSolverBody(const Body& body)
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


		static VX_INLINE void WriteBackBody(const SolverBody& local_body, Body& body)
		{
			body.SetLinearVelocity(local_body.v);
			body.SetAngularVelocity(local_body.w);
		}
		static void WriteBackBodies(const SolverBody* bodies, uint32 count, BodyManager& body_manager)
		{
			VX_PROFILE_FUNCTION();
			for (const SolverBody* sb = bodies, *sb_end = bodies + count; sb < sb_end; ++sb)
			{
				if ((*sb).invMass == 0)
					continue;
				WriteBackBody(*sb, body_manager.GetBody((*sb).bodyID));
			}
		}

		SolverBody& GetSolverBody(SolverBodyIndex local_idx)
		{
			return mBodies[local_idx.Value()];
		}

		SolverBody* GetBodiesPtr() { return mBodies.data(); }
		size_t GetBodiesCount() { return mBodies.size(); }
		Linear1DRow* GetLinearRowPtr() { return mLinear1DRows.data(); }
		size_t LinearRowCount() const { return mLinear1DRows.size(); }

		Constraint** GetConstraintResolvePositionQueuePtr() { return mConstraintPositionSolveQueue.data(); }
		size_t ConstraintResolvePositionQueueCount() const { return mConstraintPositionSolveQueue.size(); }

		void HackClear()
		{
			mLinear1DRows.clear();

			//quick hack, no caching pipeline and to prevent bugs 
			mBodies.clear();
			std::fill(mBodyToSolverBody.begin(), mBodyToSolverBody.end(), SolverBodyIndex{});
			mConstraintPositionSolveQueue.clear();
		}

		void AddLinearRow(const Linear1DRow& row)
		{
			mLinear1DRows.push_back(row);
		}

		void AppendPositionCorrectionQueue(Constraint* constraint)
		{
			mConstraintPositionSolveQueue.push_back(constraint);
		}

		static void SolverVelocityLinear1DRow(Linear1DRow& row, SolverBody* bodies)
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

		/// static for future multothreading
		static void SolverVelocityLinear1DRows(Linear1DRow* rows, size_t begin_offset, size_t count, SolverBody* bodies)
		{
			VX_PROFILE_FUNCTION();
			//for (Linear1DRow** r = rows, **r_end = rows + count; r < r_end; ++r)
			for (Linear1DRow* r = rows, *r_end = rows + count; r < r_end; ++r)
				SolverVelocityLinear1DRow(*r, bodies);
		}


		static void SolveConstraintsPosition(Constraint** constraints, size_t count, float dt, float baumgarte)
		{
			VX_PROFILE_FUNCTION();

			for (Constraint** c = constraints, **c_end = constraints + count; c < c_end; ++c)
				(*c)->SolvePositionConstraint(dt, baumgarte);
		}

		static VX_INLINE void WarmStart(const Linear1DRow& row, SolverBody& body0, SolverBody& body1)
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

			//row.lambda = 0.0f;
		}

		static void WarmStart(Linear1DRow* rows, size_t begin_offset, size_t count, SolverBody* bodies)
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

		void CommitStateConstraint()
		{
			VX_PROFILE_FUNCTION();
			for (const auto& r : mLinear1DRows)
				if (r.user)
					r.user->CommitSolverState(r);
		}


		void SolverAll(const PhysicsStepContext& ctx, uint32 iterations)
		{
			VX_PROFILE_FUNCTION();
			ConstraintSolver::WarmStart(mLinear1DRows.data(), 0, mLinear1DRows.size(), mBodies.data());

			for (int i = 0; i < iterations; ++i)
				ConstraintSolver::SolverVelocityLinear1DRows(mLinear1DRows.data(), 0, mLinear1DRows.size(), mBodies.data());
			{
				VX_PROFILE_SCOPE("ConstraintSolver Solve all commit state");
				for (const auto& r : mLinear1DRows)
					if (r.user)
						r.user->CommitSolverState(r);
			}

			ConstraintSolver::WriteBackBodies(mBodies.data(), mBodies.size(), *ctx.bodyManager);
		}
	private:
		//vectot for now
		std::vector<Linear1DRow> mLinear1DRows;
	};


} //namespace vx