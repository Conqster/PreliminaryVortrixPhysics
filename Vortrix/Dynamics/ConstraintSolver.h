#pragma once

#include "Vortrix/Maths/Vec3.h"
#include "Body/BodyID.h"
#include "SolverBodyIndex.h"

#include "Body/BodyManager.h"

#include "Constraints/Constraint.h"

#include "Vortrix/Core/Profiler.h"

#include "ConstraintSolverRow.h"

#include <mutex>

#include "Vortrix/Core/ScratchAllocator.h"

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
		
		std::mutex mSolverBodyMutex;
	public:

		void Init(const BodyManager& body_manager);

		~ConstraintSolver();

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
		SolverBodyIndex GetOrCreateSolverBody(BodyID physics_body_id, const PhysicsStepContext& ctx);

		SolverBodyIndex GetOrCreateSolverBody(const Body& body);

		SolverBodyIndex TryGetSolverBodyIndex(BodyID physics_body_id) const;


		static VX_INLINE void WriteBackBody(const SolverBody& local_body, Body& body)
		{
			body.SetLinearVelocity(local_body.v);
			body.SetAngularVelocity(local_body.w);
		}
		static void WriteBackBodies(const SolverBody* bodies, uint32 count, BodyManager& body_manager);

		SolverBody& GetSolverBody(SolverBodyIndex local_idx) { return mBodies[local_idx.Value()]; }
		const SolverBody& GetSolverBody(SolverBodyIndex local_idx) const { return mBodies[local_idx.Value()]; }

		void PrepareSolver(uint32 required_liner_row, uint32 required_position_correct_constraint, const PhysicsStepContext& ctx);

		void ReleaseAllocation(ScratchAllocator* scratchAllocator);

		SolverBody* GetBodiesPtr() { return mBodies.data(); }
		size_t GetBodiesCount() { return mBodies.size(); }
		Linear1DRow* GetLinearRowPtr() { return mLinear1DRows; }
		size_t LinearRowCount() const { return mLinear1DRowsCounts; }
		size_t Linear1DRowBufferCount() const { return mLinear1DRowBufferCount; }

		Constraint** GetConstraintResolvePositionQueuePtr() { return mConstraintPositionSolveQueue; }
		size_t ConstraintResolvePositionQueueCount() const { return mConstraintPositionSolveQueueCounts; }

		void HackClear();

		Linear1DRow* AllocateLinear1DRow(uint32 count)
		{
			//VX_ASSERT(count >= 1);
			//for(int i = 0; i<count;++i)
			//	mLinear1DRows.push_back({});
			//return &mLinear1DRows.back() - (count -1);

			VX_ASSERT(count >= 1);
			Linear1DRow* alloc = mLinear1DRows + mLinear1DRowsCounts;
			mLinear1DRowsCounts += count;
			VX_ASSERT(mLinear1DRowsCounts <= mLinear1DRowBufferCount);
			return alloc;
		}

		void AppendPositionCorrectionQueue(Constraint* constraint)
		{
			//mConstraintPositionSolveQueue.push_back(constraint);

			mConstraintPositionSolveQueue[mConstraintPositionSolveQueueCounts++] = constraint;
			VX_ASSERT(mConstraintPositionSolveQueueCounts <= mConstraintPositionSolveQueueBufferCount);
		}

		static void SolverVelocityLinear1DRow(Linear1DRow& row, SolverBody* bodies);

		/// static for future multothreading
		static void SolverVelocityLinear1DRows(Linear1DRow* rows, size_t begin_offset, size_t count, SolverBody* bodies);


		static void SolveConstraintsPosition(Constraint** constraints, size_t count, float dt, float baumgarte);

		static VX_INLINE void WarmStart(const Linear1DRow& row, SolverBody& body0, SolverBody& body1);

		static void WarmStart(Linear1DRow* rows, size_t begin_offset, size_t count, SolverBody* bodies);

		void CommitStateConstraint()
		{
			VX_PROFILE_FUNCTION();
			//for (const auto& r : mLinear1DRows)
			//	if (r.user)
			//		r.user->CommitSolverState(r);

			for (Linear1DRow* r = mLinear1DRows, *r_end = mLinear1DRows + mLinear1DRowsCounts; r < r_end; ++r)
				if ((*r).user)
					(*r).user->CommitSolverState(*r);
		}


		void SolverAll(const PhysicsStepContext& ctx, uint32 iterations);
	private:
		//vectot for now
		//std::vector<Linear1DRow> mLinear1DRows;
		Linear1DRow* mLinear1DRows;
		uint32 mLinear1DRowsCounts = 0;
		uint32 mLinear1DRowBufferCount = 0;

		Constraint** mConstraintPositionSolveQueue;
		uint32 mConstraintPositionSolveQueueCounts = 0;
		uint32 mConstraintPositionSolveQueueBufferCount = 0;
	};


} //namespace vx