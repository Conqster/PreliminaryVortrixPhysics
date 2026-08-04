#include "IslandCoordinator.h"

#include "Vortrix/Core/Atomics.h"

namespace vx {

	IslandCoordinator::~IslandCoordinator()
	{
		delete[] mActiveBodyLinkIndices;
		delete[] mContactConstraintBodyLinkIndices;

		delete[] mBodyIDIslands;
		delete[] mSolverBodyIndexIslands;
		delete[] mBodyIDPerIslandIndexEnds;

		delete[] mConstraintIndicesIslands;
		delete[] mConstraintIslandIndexEnds;

		delete[] mNonContactConstraintRowBodyLinkIndices;

		delete[] mNonConstactConstraintRowIndicesIslands;
		delete[] mNonConstraintRowIslandIndexEnds;
	}

	void IslandCoordinator::Init(uint32 max_bodies, uint32 max_contact_constraint, uint32 max_noncontact_constraint)
	{
		mActiveBodyLinkIndices = new std::atomic<uint32>[max_bodies];
		//mIslandIdxs = new uint32[max_bodies];
		mBodyIDIslands = new BodyID[max_bodies];
		mSolverBodyIndexIslands = new SolverBodyIndex[max_bodies];
		mBodyIDPerIslandIndexEnds = new uint32[max_bodies];

		mContactConstraintBodyLinkIndices = new uint32[max_contact_constraint];
		mMaxContactConstraint = max_contact_constraint;

		///later make this temporary step data; using scratch allocation
		mNonContactConstraintRowBodyLinkIndices = new ConstraintRowEdge[max_noncontact_constraint];
		mMaxNonContactConstraint = max_noncontact_constraint;

		mConstraintIndicesIslands = new uint32[max_bodies];
		mConstraintIslandIndexEnds = new uint32[max_bodies];

		mNonConstactConstraintRowIndicesIslands = new uint32[max_bodies];
		mNonConstraintRowIslandIndexEnds = new uint32[max_bodies];



		mIslandIdxs.resize(max_bodies);
	}

	void IslandCoordinator::PrepareIslands(uint32 body_count)
	{
		VX_PROFILE_FUNCTION();
		for (uint32 i = 0; i < body_count; ++i)
			mActiveBodyLinkIndices[i] = i;


		mIslandCount = { 0 };
		mValidateStepMaxConstraint = { 0 };

		mValidateStepMaxNonConstraint = { 0 };

		//mIslandIdxs.clear();
		mIslandIdxs.resize(body_count, 0);
	}

	uint32 IslandCoordinator::ComputeActiveBodyLowestIdx(uint32 idx)
	{
		//		uint32 v = mBodiesIdxs[idx];

		//while (mBodiesIdxs[v] != v)
		//	v = mBodiesIdxs[v];
		
		//return v;


		uint32 v = idx;
		while (true)
		{
			uint32 body_links_to = mActiveBodyLinkIndices[v].load(std::memory_order_relaxed);
			if (body_links_to == v)
				break;
			v = body_links_to;
		}
		return v;
	}

	void IslandCoordinator::LinkBodies(uint32 body_activeA, uint32 body_activeB)
	{
		//uint32 l0 = ComputeActiveBodyLowestIdx(body_activeA);
		//uint32 l1 = ComputeActiveBodyLowestIdx(body_activeB);
		
		//if (l0 > l1)
		//	std::swap(l0, l1);
		
		//mBodiesIdxs[l1] = l0;
		
		//mBodiesIdxs[body_activeA] = l0;
		//mBodiesIdxs[idx1] = l0;

		uint32 l0 = body_activeA;
		uint32 l1 = body_activeB;

		for (;;)
		{
			l0 = ComputeActiveBodyLowestIdx(l0);
			l1 = ComputeActiveBodyLowestIdx(l1);

			if (l0 != l1)
			{

				if (l0 < l1)
				{
					if (!mActiveBodyLinkIndices[l1].compare_exchange_weak(l1, l0, std::memory_order_relaxed))
						continue;
				}
				else
				{
					if (!mActiveBodyLinkIndices[l0].compare_exchange_weak(l0, l1, std::memory_order_relaxed))
						continue;
				}
			}

			uint32 lowest_link = VxMin(l0, l1);
			atomic::Min(mActiveBodyLinkIndices[body_activeA], lowest_link, std::memory_order_relaxed);
			atomic::Min(mActiveBodyLinkIndices[body_activeB], lowest_link, std::memory_order_relaxed);
			break;
		}
	}



	void IslandCoordinator::LinkContactConstraint(uint32 constraint_idx, uint32 min_active_body_idx)
	{
		VX_ASSERT(constraint_idx < mMaxContactConstraint);
		VX_ASSERT(min_active_body_idx != Body::kInvalidActiveIdx);

		atomic::Max(mValidateStepMaxConstraint, constraint_idx + 1);
		mContactConstraintBodyLinkIndices[constraint_idx] = min_active_body_idx;
	}

	void IslandCoordinator::LinkNonConstactConstraint(uint32 constraint_row_idx, uint32 row_count, uint32 min_active_body_idx)
	{
		VX_ASSERT(constraint_row_idx < mMaxNonContactConstraint);
		VX_ASSERT(min_active_body_idx != Body::kInvalidActiveIdx);

		atomic::Max(mValidateStepMaxNonConstraint, constraint_row_idx + row_count);

		mNonContactConstraintRowBodyLinkIndices[constraint_row_idx] = { min_active_body_idx, constraint_row_idx, row_count };
	}

	void IslandCoordinator::FinaliseIslands(const ConstraintSolver& constraint_solver, uint32 contact_constraint_count, 
		uint32 active_non_contact_constraint_count, BodyManager& body_manager, ScratchAllocator* scratchAllocator)
	{
		const uint32 active_bodies_count = body_manager.GetNumActiveBodies();
		if (active_bodies_count <= 0) return;
		uint32* temp_active_bodies_island_indices = reinterpret_cast<uint32*>(scratchAllocator->Allocate(sizeof(uint32) * active_bodies_count));
		FinaliseBodyIslands(constraint_solver, body_manager, active_bodies_count, temp_active_bodies_island_indices, scratchAllocator);
		FinaliseContactConstraint(contact_constraint_count, mIslandCount, temp_active_bodies_island_indices, scratchAllocator);
		FinaliseNonContactConstraint(active_non_contact_constraint_count, mIslandCount, temp_active_bodies_island_indices, scratchAllocator);

		scratchAllocator->Free(temp_active_bodies_island_indices, sizeof(uint32) * active_bodies_count);
	}

	void IslandCoordinator::FinaliseBodyIslands(const ConstraintSolver& constraint_solver, BodyManager& body_manager, const uint32 active_bodies_count, uint32* io_temp_active_bodies_island_indices, ScratchAllocator* scratchAllocator)
	{
		uint32 next_island_idx = 0;
		

		if (active_bodies_count == 0) return;

		/////update island idxs 
		//for (uint32 i = 0; i < active_bodies_count; ++i)
		//{
		//	uint32 body_links = mBodiesIdxs[i];

		//	///links with self
		//	if (body_links == i)
		//		mIslandIdxs[i] = next_island_idx++;
		//	else
		//	{
		//		///not with self, Find 
		//		uint32 v = ComputeActiveBodyLowestIdx(body_links);
		//		if (v < i) //if behind, left side, already solve 
		//			mIslandIdxs[i] = mIslandIdxs[v];
		//		/// its ahead and need to be resolved
		//		else if (v == i) //another self link
		//		{
		//			mIslandIdxs[i] = next_island_idx++;
		//		}
		//		else
		//		{
		//			VX_LOG_INFO("Propably need to link with self");
		//		}
		//	}
		//	//if (body_links == i)
		//	//	mIslandIdxs[i] = next_island_idx++;
		//	//else
		//	//{
		//	//	//if (body_links < i) //if behind, left side, already solve 
		//	//	//	mIslandIdxs[i] = mIslandIdxs[body_links];
		//	//	//else
		//	//	//{
		//	//	//	VX_LOG_INFO("Propably need to link with self");

		//	//	//}
		//	//
		//	//	///not with self, Find 
		//	//	uint32 v = Find(body_links);
		//	//	if (v < i) //if behind, left side, already solve 
		//	//		mIslandIdxs[i] = mIslandIdxs[v];
		//	//	else
		//	//	{
		//	//		VX_LOG_INFO("Propably need to link with self");

		//	//	}
		//	//}


		//	//SolverBody& solver_body = constraint_solver.GetSolverBody(SolverBodyIndex(i));
		//	//body_manager.GetBody(active_bodies[i]).SetIslandIndex(mIslandIdxs[i]);
		//	body_manager.GetBody(body_manager.GetActiveBodyID(i)).SetIslandIndex(mIslandIdxs[i]);
		//}

		//if (islands.size() < next_island_idx)
		//	islands.resize(next_island_idx);


		//for (auto& _island : islands)
		//	_island.bodyIds.clear();

		//for (uint32 i = 0; i < active_bodies_count; ++i)
		//{
		//	auto& body = body_manager.GetBody(body_manager.GetActiveBodyID(i));
		//	uint32 idx = body.GetIslandIndex();
		//	islands[idx].bodyIds.push_back(body.GetID());

		//}



		/// body id island 
		/// compute required size

		//uint32* island_body_start = reinterpret_cast<uint32*>(scratchAllocator->Allocate(sizeof(uint32) * islands.size()));
		//uint32 prev_start = 0;
		////first body starts at zero
		//for (uint32 i = 1; i < islands.size(); ++i)
		//{

		//}


		uint32* body_per_island = reinterpret_cast<uint32*>(scratchAllocator->Allocate(sizeof(uint32) * active_bodies_count));

		///update island idxs 
		for (uint32 i = 0; i < active_bodies_count; ++i)
		{
			uint32 body_links = mActiveBodyLinkIndices[i].load(std::memory_order_relaxed);

			///links with self
			if (body_links == i)
			{
				//new island start at one
				body_per_island[next_island_idx] = 1;
				mIslandIdxs[i] = next_island_idx++;
			}
			else
			{
				///not with self, Find 
				uint32 v = ComputeActiveBodyLowestIdx(body_links);
				if (v < i) //if behind, left side, already solve 
					mIslandIdxs[i] = mIslandIdxs[v];
				/// its ahead and need to be resolved
				else
				{
					VX_ASSERT(false);
					VX_LOG_INFO("Propably need to link with self");
				}

				body_per_island[mIslandIdxs[i]]++;
			}
			body_manager.GetBody(body_manager.GetActiveBodyID(i)).SetIslandIndex(mIslandIdxs[i]);
			io_temp_active_bodies_island_indices[i] = mIslandIdxs[i];
		}

		mIslandCount = next_island_idx;

		/// old island building
#pragma region Old Island Building
		//if (islands.size() < next_island_idx)
		//	islands.resize(next_island_idx);


		//for (auto& _island : islands)
		//	_island.bodyIds.clear();

		//for (uint32 i = 0; i < active_bodies_count; ++i)
		//{
		//	auto& body = body_manager.GetBody(body_manager.GetActiveBodyID(i));
		//	uint32 idx = body.GetIslandIndex();
		//	islands[idx].bodyIds.push_back(body.GetID());
		//}
#pragma endregion

#define ALLOW_DEBUG_ISLAND 0
#if ALLOW_DEBUG_ISLAND
#define DEBUG_ISLAND(x) x
#else
#define DEBUG_ISLAND(...)
#endif // ALLOW_DEBUG_ISLAND

		

		uint32* island_start_write = reinterpret_cast<uint32*>(scratchAllocator->Allocate(sizeof(uint32) * mIslandCount));
		/// first island is 0 
		island_start_write[0] = 0;


		DEBUG_ISLAND(
			/// procedding island start from the size/count of the body in previous island
			StackString island_start_idx_txt("Island start idx: ");
		StackString body_count_per_island("Island Body Count: ");
		island_start_idx_txt << island_start_write[0] << ", ";
		body_count_per_island << body_per_island[0] << ", ";);

		for (uint32 i = 1; i < next_island_idx; ++i)
		{
			/// num of bodies in previous island + start of previous island 
			uint32 body_per_prev_island = body_per_island[i - 1];
			uint32 prev_island_start = island_start_write[i - 1];
			island_start_write[i] = prev_island_start + body_per_prev_island;

			DEBUG_ISLAND(
				island_start_idx_txt << island_start_write[i] << ", ";
			body_count_per_island << body_per_island[i] << ", ";);
		}
		DEBUG_ISLAND(
			VX_LOG_INFO("---------------------------------------");
		VX_LOG_INFO(island_start_idx_txt);
		VX_LOG_INFO(body_count_per_island););



#if ALLOW_DEBUG_ISLAND
		StackString active_bodies_island;
		StackString writing_spendle;
#endif // ALLOW_DEBUG_ISLAND

		for (uint32 i = 0; i < active_bodies_count; ++i)
		{

			uint32 active_body_island_idx = io_temp_active_bodies_island_indices[i];
			DEBUG_ISLAND(active_bodies_island << active_body_island_idx << ", ";);


			uint32& write_spendle = island_start_write[active_body_island_idx];

			DEBUG_ISLAND(writing_spendle << write_spendle << ", ";);

			const BodyID& body_id = body_manager.GetActiveBodyID(i);
			mBodyIDIslands[write_spendle] = body_id;
			mSolverBodyIndexIslands[write_spendle] = constraint_solver.TryGetSolverBodyIndex(body_id);
			//VX_ASSERT(mSolverBodyIndexIslands[write_spendle].IsValid());
			write_spendle++;

		}

		for (uint32 i = 0; i < next_island_idx; ++i)
			mBodyIDPerIslandIndexEnds[i] = island_start_write[i];



		//// Validate 
		for (uint32 i = 0; i < active_bodies_count; ++i)
		{
			if (mSolverBodyIndexIslands[i].IsValid())
			{
				const SolverBody& solver_body = constraint_solver.GetSolverBody(mSolverBodyIndexIslands[i]);
				VX_ASSERT(solver_body.bodyID == mBodyIDIslands[i], "solver body index body id validation failed");
			}
		}


		DEBUG_ISLAND(
			VX_LOG_INFO("Active Body: ", active_bodies_island);
		VX_LOG_INFO("Write Body: ", writing_spendle);
		island_start_idx_txt.Clear();
		island_start_idx_txt << "Island Update start: ";
		for (uint32 i = 0; i < next_island_idx; ++i)
			island_start_idx_txt << island_start_write[i] << ", ";
		VX_LOG_INFO(island_start_idx_txt);
			);

		scratchAllocator->Free(island_start_write, sizeof(uint32) * mIslandCount);
		scratchAllocator->Free(body_per_island, sizeof(uint32) * active_bodies_count);
	}

	void IslandCoordinator::FinaliseContactConstraint(const uint32 constraint_count, const uint32 island_count, const uint32* i_temp_active_bodies_island_indices, ScratchAllocator* scratchAllocator)
	{
		if (constraint_count <= 0) return;

		VX_ASSERT(mValidateStepMaxConstraint == constraint_count);
		uint32* constraint_per_island = reinterpret_cast<uint32*>(scratchAllocator->Allocate(sizeof(uint32) * island_count));

		for (uint32 i = 0; i < island_count; ++i)
			constraint_per_island[i] = 0;


		/// derive constraint island slot from bodies idx 
		/// bodies idx already have island allocated 

		///update island idxs 
		for (uint32 i = 0; i < constraint_count; ++i)
		{
			uint32 constraint_body_idx = mContactConstraintBodyLinkIndices[i];
			uint32 active_body_idx_island = i_temp_active_bodies_island_indices[constraint_body_idx];

			constraint_per_island[active_body_idx_island]++;
		}


		uint32* island_start_write = reinterpret_cast<uint32*>(scratchAllocator->Allocate(sizeof(uint32) * island_count));
		/// first island is 0 
		island_start_write[0] = 0;

		for (uint32 i = 1; i < island_count; ++i)
		{
			/// num of bodies in previous island + start of previous island 
			uint32 body_per_prev_island = constraint_per_island[i - 1];
			uint32 prev_island_start = island_start_write[i - 1];
			island_start_write[i] = prev_island_start + body_per_prev_island;
		}


		for (uint32 constraint_idx = 0; constraint_idx < constraint_count; ++constraint_idx)
		{

			uint32 constraint_body_idx = mContactConstraintBodyLinkIndices[constraint_idx];
			uint32 active_body_idx_island = i_temp_active_bodies_island_indices[constraint_body_idx];

			uint32& write_spendle = island_start_write[active_body_idx_island];

			mConstraintIndicesIslands[write_spendle] = constraint_idx;
			write_spendle++; ///increment island write spendle

		}

		/// update constraiunt end with island incremneted write
		for (uint32 i = 0; i < island_count; ++i)
			mConstraintIslandIndexEnds[i] = island_start_write[i];


		//// Validate; get constraint from contact constrain solver then get body a then gets its island if it matches 
		//for (uint32 i = 0; i < constraint_count; ++i)
		//{

		//	if (mSolverBodyIndexIslands[i].IsValid())
		//	{
		//		const SolverBody& solver_body = constraint_solver.GetSolverBody(mSolverBodyIndexIslands[i]);
		//		VX_ASSERT(solver_body.bodyID == mBodyIDIslands[i], "solver body index body id validation failed");
		//	}
		//}


		scratchAllocator->Free(island_start_write, sizeof(uint32) * island_count);
		scratchAllocator->Free(constraint_per_island, sizeof(uint32) * island_count);
	}

	void IslandCoordinator::FinaliseNonContactConstraint(const uint32 constraint_count, const uint32 island_count, const uint32* i_temp_active_bodies_island_indices, ScratchAllocator* scratchAllocator)
	{
		if (constraint_count <= 0) return;

		VX_ASSERT(mValidateStepMaxNonConstraint == constraint_count);
		//technically this is constraint row
		uint32* constraint_per_island = reinterpret_cast<uint32*>(scratchAllocator->Allocate(sizeof(uint32) * island_count));

		std::memset(constraint_per_island, 0, island_count * sizeof(ConstraintRowEdge));

		/// derive constraint island slot from bodies idx 
		/// bodies idx already have island allocated 
		
		///update island idxs 
		for (uint32 i = 0; i < constraint_count;)
		{

			/// this is the non constriant island access from constaint solveer 
			/// unlike contact solver; non contact uses mathemarice representation of constraint as jacobian row
			/// i.e write spendle increament is row start + count 
			ConstraintRowEdge constraint_row_edge = mNonContactConstraintRowBodyLinkIndices[i];
			uint32 active_body_idx_island = i_temp_active_bodies_island_indices[constraint_row_edge.bodyLink];

			constraint_per_island[active_body_idx_island] += constraint_row_edge.rowCount;

			//increament
			i += constraint_row_edge.rowCount; ///increment based on row count 
		}



		uint32* island_start_write = reinterpret_cast<uint32*>(scratchAllocator->Allocate(sizeof(uint32) * island_count));
		/// first island is 0 
		island_start_write[0] = 0;

		///constraint absolute 
		for (uint32 i = 1; i < island_count; ++i)
		{
			/// num of bodies in previous island + start of previous island 
			uint32 body_per_prev_island = constraint_per_island[i - 1];
			uint32 prev_island_start = island_start_write[i - 1];
			island_start_write[i] = prev_island_start + body_per_prev_island;
		}

		for (uint32 constraint_idx = 0; constraint_idx < constraint_count;)// ++constraint_idx)
		{

			ConstraintRowEdge constraint_row_edge = mNonContactConstraintRowBodyLinkIndices[constraint_idx];
			uint32 active_body_idx_island = i_temp_active_bodies_island_indices[constraint_row_edge.bodyLink];

			uint32& write_spendle = island_start_write[active_body_idx_island];


			/// rows for this constraint 
			/// rows per constraint a contigous 
			for (uint32 row = 0; row < constraint_row_edge.rowCount; ++row)
			{
				mNonConstactConstraintRowIndicesIslands[write_spendle] = constraint_idx + row;
				write_spendle++; ///increment island write spendle
			}

			//increament
			constraint_idx += constraint_row_edge.rowCount; ///increment based on row count 
		}


		/// update constraiunt end with island incremneted write
		for (uint32 i = 0; i < island_count; ++i)
			mNonConstraintRowIslandIndexEnds[i] = island_start_write[i];

		scratchAllocator->Free(island_start_write, sizeof(uint32) * island_count);
		scratchAllocator->Free(constraint_per_island, sizeof(uint32) * island_count);
	}




	void IslandCoordinator::SortIslands()
	{
	}

	IslandCoordinator::IslandRange<BodyID> IslandCoordinator::IslandBodyIDsRange(uint32 island_idx) const
	{
		uint32 start_idx = (island_idx != 0) ? mBodyIDPerIslandIndexEnds[island_idx - 1] : 0;
		uint32 end_idx = mBodyIDPerIslandIndexEnds[island_idx];

		return IslandRange<BodyID>(
			&mBodyIDIslands[start_idx], 
			&mBodyIDIslands[end_idx]
		);
	}

	IslandCoordinator::IslandRange<SolverBodyIndex> IslandCoordinator::IslandSolverBodyIndicesRange(uint32 island_idx) const
	{
		uint32 start_idx = (island_idx != 0) ? mBodyIDPerIslandIndexEnds[island_idx - 1] : 0;
		uint32 end_idx = mBodyIDPerIslandIndexEnds[island_idx];

		return IslandRange<SolverBodyIndex>(
			&mSolverBodyIndexIslands[start_idx],
			&mSolverBodyIndexIslands[end_idx]
		);
	}

	IslandCoordinator::IslandRange<uint32> IslandCoordinator::IslandContactConstraintIndicesRange(uint32 island_idx) const
	{
		if (mValidateStepMaxConstraint.load(std::memory_order_relaxed) <= 0) return IslandRange<uint32>(nullptr, nullptr);
		uint32 start_idx = (island_idx != 0) ? mConstraintIslandIndexEnds[island_idx - 1] : 0;
		uint32 end_idx = mConstraintIslandIndexEnds[island_idx];

		return IslandRange<uint32>(
			&mConstraintIndicesIslands[start_idx],
			&mConstraintIndicesIslands[end_idx]
		);
	}

	IslandCoordinator::IslandRange<uint32> IslandCoordinator::IslandNonContactConstraintRowIndicesRange(uint32 island_idx) const
	{
		if (mValidateStepMaxNonConstraint.load(std::memory_order_relaxed) <= 0) return IslandRange<uint32>(nullptr, nullptr);
		uint32 start_idx = (island_idx != 0) ? mNonConstraintRowIslandIndexEnds[island_idx - 1] : 0;
		uint32 end_idx = mNonConstraintRowIslandIndexEnds[island_idx];

		return IslandRange<uint32>(
			&mNonConstactConstraintRowIndicesIslands[start_idx],
			&mNonConstactConstraintRowIndicesIslands[end_idx]
		);
	}



} ///namespace vx 

