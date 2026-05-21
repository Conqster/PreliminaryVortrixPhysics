#pragma once
#include "ConstraintSolverRow.h"

#include "Constraint.h"
#include "DistanceConstraint.h"

#include "PhysicsWorldSettings.h"
#include "Body/BodyManager.h"

class DebugGizmosRenderer;

namespace vx {



	class ConstraintCoordinator
	{
	public:
		void AddConstraint(Constraint* constraint)
		{
			mConstraints.push_back(constraint);
		}

		//void AddConstraint(const DistanceConstraint& joint)
		//{
		//	AddConstraint(new DistanceConstraint(joint));
		//}

		template<typename T>
		T* AddConstraintT(const T& joint)
		{
			T* _j = new T(joint);
			AddConstraint(_j);
			return _j;
		}

		std::vector<Constraint*>& GetConstraints() { return mConstraints; }

		void PrepConstraintSolving(ConstraintSolver& solver, const PhysicsStepContext& ctx)
		{
			for (auto& c : mConstraints)
				c->PrepSolver(&solver, ctx);
		}


		void DebugGizmos(DebugGizmosRenderer* debug_renderer)
		{
			for (auto& c : mConstraints)
				c->DebugGizmos(debug_renderer);
		}
	private: 
		//for now vector 
		std::vector<Constraint*> mConstraints;
	};


	struct SolverBody
	{
		Vec3 linearVelocity;
		Vec3 angularVelocity;
		float invMass;
		BodyID bodyID;
	};

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

	public:

		void Init(const BodyManager& body_manager)
		{
			uint32 max_bodies = body_manager.MaxBodies();
			mBodies.reserve(max_bodies);
			mBodyToSolverBody.resize(max_bodies);
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
			SolverBodyIndex& solver_idx = mBodyToSolverBody[physics_body_id.Value()];

			//if (solver_idx.Value() >= 0)
			if (solver_idx.IsValid())
				return solver_idx;


			const Body& body = ctx.bodyManager->GetBody(physics_body_id);

			SolverBody solver_body;
			solver_body.bodyID = physics_body_id;
			solver_body.linearVelocity = body.GetLinearVelocity();
			solver_body.angularVelocity = body.GetAngularVelocity();
			solver_body.invMass = body.GetInverseMass();

			uint32 new_idx = uint32(mBodies.size());
			mBodies.push_back(solver_body);

			solver_idx = SolverBodyIndex(new_idx);
			//mBodyToSolverBody[physics_body_id.Value()] = new_idx;

			return solver_idx;
		}


		static VX_INLINE void WriteBackBody(const SolverBody& local_body, Body& body)
		{
			body.SetLinearVelocity(local_body.linearVelocity);
			body.SetAngularVelocity(local_body.angularVelocity);
		}
		static void WriteBackBodies(const SolverBody* bodies, uint32 count, BodyManager& body_manager)
		{
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


		void HackClear()
		{
			mLinear1DRows.clear();

			//quick hack, no caching pipeline and to prevent bugs 
			mBodies.clear();
			std::fill(mBodyToSolverBody.begin(), mBodyToSolverBody.end(), SolverBodyIndex{});
		}

		void AddLinearRow(const Linear1DRow& row)
		{
			mLinear1DRows.push_back(row);
		}
		
		static void SolverVelocityLinear1DRow(Linear1DRow& row, SolverBody* bodies)
		{
			if (row.effectiveMass <= 0.0f)
				return;

			SolverBody& sbA = bodies[row.bodyAidx.Value()];
			SolverBody& sbB = bodies[row.bodyBidx.Value()];

			bool dyn_a = true;
			bool dyn_b = true;


			///this is the work of solver body and not direct for Body
			Vec3 lin_velA = sbA.linearVelocity;
			Vec3 ang_velA = sbA.angularVelocity;
			float inv_massA = sbA.invMass;

			Vec3 lin_velB = sbB.linearVelocity;
			Vec3 ang_velB = sbB.angularVelocity;
			float inv_massB = sbB.invMass;


			VX_ASSERT(!lin_velA.IsNaN(), "lin_velA is nan");
			VX_ASSERT(!ang_velA.IsNaN(), "ang_velA is nan");
			VX_ASSERT(!lin_velB.IsNaN(), "lin_velB is nan");
			VX_ASSERT(!ang_velB.IsNaN(), "ang_velB is nan");


			//load data
			Vec3 rAXn = Vec3::LoadFloat3Raw(row.angularA);
			Vec3 invIrAXn = Vec3::LoadFloat3Raw(row.invIAngularA);

			Vec3 rBXn = Vec3::LoadFloat3Raw(row.angularB);
			Vec3 invIrBXn = Vec3::LoadFloat3Raw(row.invIAngularB);


			//then later in constraint solver 

			//jacobian 
			float jv;
			if (dyn_a && dyn_b) ///if constexpr (
				jv = (lin_velA - lin_velB).Dot(row.axis);
			else if (dyn_a)
				jv = lin_velA.Dot(row.axis);
			else if (dyn_b)
				jv = (-lin_velB).Dot(row.axis);
			else
			{
				VX_LOG_ERROR("Static vs static this should not be possible");
				jv = 0.0f;
			}

			if (dyn_a)
				jv += rAXn.Dot(ang_velA);
			if (dyn_b)
				jv -= rBXn.Dot(ang_velB);

			//float actual_bias = 
			float lambda = (jv - row.bias) * row.effectiveMass;
			float _lambda = VxClamp(row.lambda + lambda, row.minLambda, row.maxLambda);
			float impluse = _lambda - row.lambda;
			row.lambda = _lambda;
			//updated jn

			//store changes
			if (dyn_a)
			{
				lin_velA -= impluse * inv_massA * row.axis;
				ang_velA -= impluse * invIrAXn;
			}
			if (dyn_b)
			{
				lin_velB += impluse * inv_massB * row.axis;
				ang_velB += impluse * invIrBXn;
			}


			VX_ASSERT(!lin_velA.IsNaN(), "lin_velA is nan");
			VX_ASSERT(!ang_velA.IsNaN(), "ang_velA is nan");
			VX_ASSERT(!lin_velB.IsNaN(), "lin_velB is nan");
			VX_ASSERT(!ang_velB.IsNaN(), "ang_velB is nan");

			//write back to body 
			sbA.linearVelocity = lin_velA;
			sbA.angularVelocity = ang_velA;

			sbB.linearVelocity = lin_velB;
			sbB.angularVelocity = ang_velB;
		}

		/// static for future multothreading
		static void SolverVelocityLinear1DRows(Linear1DRow* rows, size_t begin_offset, size_t count, SolverBody* bodies)
		{
			//for (Linear1DRow** r = rows, **r_end = rows + count; r < r_end; ++r)
			for (Linear1DRow* r = rows, *r_end = rows + count; r < r_end; ++r)
				SolverVelocityLinear1DRow(*r, bodies);
		}

		static VX_INLINE void WarmStart(Linear1DRow& row, SolverBody& body0, SolverBody& body1)
		{
			if (row.lambda == 0.0f)
				return;

			//row.lambda *= 0.8f;
			float impluse = row.lambda;

			body0.linearVelocity -= impluse * body0.invMass * row.axis;
			body0.angularVelocity -= impluse * Vec3::LoadFloat3Raw(row.invIAngularA);

			body1.linearVelocity += impluse * body1.invMass * row.axis;
			body1.angularVelocity += impluse * Vec3::LoadFloat3Raw(row.invIAngularB);

			//row.lambda = 0.0f;
		}

		static void WarmStart(Linear1DRow* rows, size_t begin_offset, size_t count, SolverBody* bodies)
		{
			for (Linear1DRow* r = rows, *r_end = rows + count; r < r_end; ++r)
			{
				Linear1DRow& row = (*r);
				SolverBody& sbA = bodies[row.bodyAidx.Value()];
				SolverBody& sbB = bodies[row.bodyBidx.Value()];

				WarmStart(row, sbA, sbB);
			}
		}


		void SolverAll(const PhysicsStepContext& ctx, uint32 iterations)
		{
			ConstraintSolver::WarmStart(mLinear1DRows.data(), 0, mLinear1DRows.size(), mBodies.data());

			for (int i = 0; i < iterations; ++i)
				ConstraintSolver::SolverVelocityLinear1DRows(mLinear1DRows.data(), 0, mLinear1DRows.size(), mBodies.data());

			for (const auto& r : mLinear1DRows)
				if (r.user)
					r.user->CommitSolverState(r);

			ConstraintSolver::WriteBackBodies(mBodies.data(), mBodies.size(), *ctx.bodyManager);
		}
	private:
		//vectot for now
		std::vector<Linear1DRow> mLinear1DRows;
	};




} //namespace vx