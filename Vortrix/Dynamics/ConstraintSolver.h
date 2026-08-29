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

		const Body& AttemptGetBody(const BodyManager* body_manager, SolverBodyIndex local_idx) const
		{
			VX_ASSERT(body_manager && local_idx.IsValid());

			const SolverBody& solver_body = GetSolverBody(local_idx);

			return body_manager->GetBody(solver_body.bodyID);
		}

		const Body* AttemptGetBodyPtr(const BodyManager* body_manager, SolverBodyIndex local_idx) const
		{
			VX_ASSERT(body_manager && local_idx.IsValid());

			const SolverBody& solver_body = GetSolverBody(local_idx);

			return &body_manager->GetBody(solver_body.bodyID);
		}

		//Body& AttemptGetBody(BodyManager* body_manager, SolverBodyIndex local_idx)
		//{
		//	VX_ASSERT(body_manager && local_idx.IsValid());

		//	const SolverBody& solver_body = GetSolverBody(local_idx);

		//	return body_manager->GetBody(solver_body.bodyID);
		//}

		const void AttemptGetLinear1DRowBodies(const BodyManager* body_manager, uint32 idx, const Body*& body_a, const Body*& body_b) const
		{ 
			VX_ASSERT(idx < mLinear1DRowsCounts); 

			const auto& linear_row = mLinear1DRows[idx];

			body_a = &AttemptGetBody(body_manager, linear_row.bodyAidx);
			body_b = &AttemptGetBody(body_manager, linear_row.bodyBidx);
		}

		void PrepareSolver(uint32 required_liner_row, uint32 required_position_correct_constraint, const PhysicsStepContext& ctx);

		void ReleaseAllocation(ScratchAllocator* scratchAllocator);

		SolverBody* GetBodiesPtr() { return mBodies.data(); }
		size_t GetBodiesCount() { return mBodies.size(); }
		Linear1DRow* GetLinearRowPtr() { return mLinear1DRows; }
		const Linear1DRow* GetLinearRowPtr() const{ return mLinear1DRows; }
		size_t LinearRowCount() const { return mLinear1DRowsCounts; }
		size_t Linear1DRowBufferCount() const { return mLinear1DRowBufferCount; }

		Constraint** GetConstraintResolvePositionQueuePtr() { return mConstraintPositionSolveQueue; }
		size_t ConstraintResolvePositionQueueCount() const { return mConstraintPositionSolveQueueCounts; }

		void HackClear();

		Linear1DRow* AllocateLinear1DRow(uint32& out_row_start, uint32 count)
		{
			//VX_ASSERT(count >= 1);
			//for(int i = 0; i<count;++i)
			//	mLinear1DRows.push_back({});
			//return &mLinear1DRows.back() - (count -1);

			VX_ASSERT(count >= 1);
			out_row_start = mLinear1DRowsCounts;
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
		static void SolverVelocityLinear1DRow(Linear1DRow& row, SolverBody& sbA, SolverBody& sbB);

		/// static for future multothreading
		static void SolverVelocityLinear1DRows(Linear1DRow* rows, size_t begin_offset, size_t count, SolverBody* bodies);

		void SolverVelocityLinear1DRowsIndices(const uint32* rows_indices, size_t count);


		static void SolveConstraintsPosition(Constraint** constraints, size_t count, float dt, float baumgarte);

		static VX_INLINE void WarmStart(const Linear1DRow& row, SolverBody& body0, SolverBody& body1);

		static void WarmStart(Linear1DRow* rows, size_t begin_offset, size_t count, SolverBody* bodies);


		static void WarmStart(const uint32* rows_indices, uint32 indices_count,
			const Linear1DRow* linear_row_buff, uint32 total,
			SolverBody* solver_bodies, uint32 total_solver_bodies);


		static void CommitStateConstraint(const uint32* rows_indices, uint32 indices_count, 
			const Linear1DRow* linear_row_buff, uint32 total,
			Constraint** constraints, uint32 total_constraint)
		{
			for (const uint32* idx = rows_indices, *idx_end = rows_indices + indices_count; idx < idx_end; ++idx)
			{
				VX_ASSERT((*idx) < total);
				const Linear1DRow& row = linear_row_buff[(*idx)];

				VX_ASSERT(row.info.ConstraintIndex() < total_constraint);
				constraints[row.info.ConstraintIndex()]->CommitSolverState(row);
			}
		}

		void CommitStateConstraint(Constraint** constraints, uint32 available_count)
		{
			VX_PROFILE_FUNCTION();
			for (Linear1DRow* r = mLinear1DRows, *r_end = mLinear1DRows + mLinear1DRowsCounts; r < r_end; ++r)
			{
				VX_ASSERT(r->info.ConstraintIndex() < available_count);
				constraints[r->info.ConstraintIndex()]->CommitSolverState(*r);
			}
		}


		void SolverAll(const PhysicsStepContext& ctx, uint32 iterations, Constraint** constraints, uint32 count);
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