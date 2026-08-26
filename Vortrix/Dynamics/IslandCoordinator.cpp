#include "IslandCoordinator.h"

#include "Vortrix/Core/Atomics.h"
#include "ConstraintSolver/ContactConstraintSolver.h"


#include <unordered_set>

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
		VX_PROFILE_FUNCTION();
		const uint32 active_bodies_count = body_manager.NumActiveBodies();
		if (active_bodies_count <= 0) return;

		uint32* temp_active_bodies_island_indices = reinterpret_cast<uint32*>(scratchAllocator->Allocate(sizeof(uint32) * active_bodies_count));
		FinaliseBodyIslands(constraint_solver, body_manager, active_bodies_count, temp_active_bodies_island_indices, scratchAllocator);
		FinaliseContactConstraint(contact_constraint_count, mIslandCount, temp_active_bodies_island_indices, scratchAllocator);
		FinaliseNonContactConstraint(active_non_contact_constraint_count, mIslandCount, temp_active_bodies_island_indices, scratchAllocator);

		scratchAllocator->Free(temp_active_bodies_island_indices, sizeof(uint32) * active_bodies_count);

		SortIslands(scratchAllocator);
	}

	void IslandCoordinator::FinaliseBodyIslands(const ConstraintSolver& constraint_solver, BodyManager& body_manager, const uint32 active_bodies_count, uint32* io_temp_active_bodies_island_indices, ScratchAllocator* scratchAllocator)
	{
		VX_PROFILE_FUNCTION();
		uint32 next_island_idx = 0;
		

		if (active_bodies_count == 0) return;

		uint32* body_per_island = reinterpret_cast<uint32*>(scratchAllocator->Allocate(sizeof(uint32) * active_bodies_count));

		//// Mapping active body index to island index 
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
		//	islands[idx].bodyIds.push_back(body.ID());
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

		/// debugging the island construction
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
		VX_PROFILE_FUNCTION();
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
		VX_PROFILE_FUNCTION();
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


	void IslandCoordinator::SortIslands(ScratchAllocator* scratchAllocator)
	{
		if(mSortedIslandIndices.size() < mIslandCount)
			mSortedIslandIndices.resize(mIslandCount);


		uint32* island_sizes = reinterpret_cast<uint32*>(scratchAllocator->Allocate(sizeof(uint32) * mIslandCount));

		for (vx::uint32 island = 0; island < mIslandCount; ++island)
		{
			vx::IslandCoordinator::IslandRange<uint32> contact_island = ContactConstraintIndicesIslandRange(island);
			vx::IslandCoordinator::IslandRange<uint32> non_contact_island = IslandNonContactConstraintRowIndicesRange(island); ///later its better to get actual constraint

			uint32 constraint_count = 0;
			if (contact_island.Valid())
				constraint_count += contact_island.Size();

			if (non_contact_island.Valid())
				constraint_count += non_contact_island.Size();

			island_sizes[island] = constraint_count;
			mSortedIslandIndices[island] = island;
		}


		std::sort(mSortedIslandIndices.begin(), mSortedIslandIndices.begin() + mIslandCount, [island_sizes](int lhs, int rhs)
			{
				return island_sizes[lhs] > island_sizes[rhs];
			});





		scratchAllocator->Free(island_sizes, sizeof(uint32) * mIslandCount);
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

	IslandCoordinator::IslandRange<uint32> IslandCoordinator::ContactConstraintIndicesIslandRange(uint32 island_idx) const
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


	////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SPLITTER :: ISLANDSPLITBIN
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	IslandCoordinator::Splitter::EStatus IslandCoordinator::Splitter::IslandSplitBins2::NextConstactConstraintBatchRange(
		uint32& o_contact_start, uint32& o_contact_end,
		uint32& o_noncontact_start, uint32& o_noncontact_end, int& first_iteration, uint32& debug_bin)
	{
		/// Criterion to define complete
		///
		/// if nxt at end of curr bin AND last
		/// 
		/// 
		
		///was this island splitted and
		/// no bin so determine as complete
		if ((mNumActiveBins <= 0 && mBins[kMaxBin].TotalConstraintCount() <= 0) || mComplete.load(std::memory_order_relaxed))
			return EStatus::Complete;

		///
		uint64 curr_bin_next = mCurrBinNext.load(std::memory_order_acquire);

		uint32 curr_bin = GetCurrBin(curr_bin_next);
		uint32 next_batch = GetNextBatch(curr_bin_next);

		debug_bin = curr_bin;

		BinConstraintsOffsetRange bin_range = mBins[curr_bin];
		VX_ASSERT(bin_range.TotalConstraintCount() > 0);


		if (next_batch == bin_range.TotalConstraintCount())
		{
			/// if another thread mark this bin as processed then the total batch for bin equals bin_range.TotalConstraintCount()
			if (mTotalBatchProcessed.load(std::memory_order_relaxed) >= bin_range.TotalConstraintCount())
				return EStatus::Complete;

			return EStatus::WaitingForBatches; /// a thread is working on the bin; just wait and pick other batches from a different island if available
		}


		if (curr_bin == kMaxBin)
		{
			if (next_batch == 0)
			{
				/// if we get here the non parallel bin might be available
					/// CAS, if another thread beat us to the retrival of this bin 
					/// 
				uint64 new_curr_bin_next = MakeCurrBinNext(curr_bin, next_batch + bin_range.TotalConstraintCount());
				if (mCurrBinNext.compare_exchange_weak(curr_bin_next, new_curr_bin_next))
				{
					o_contact_start = bin_range.contactStart;
					o_contact_end = bin_range.contactEnd;

					o_noncontact_start = bin_range.nonContactStart;
					o_noncontact_end = bin_range.nonContactEnd;

					//first_iteration = (mIterations == 0);
					first_iteration = mIterations;

					/// retrieved 
					return EStatus::RetrievedBatch;
				}
				
				//// another thread beat us
			}
			//// another thread beat us
			return EStatus::WaitingForBatches; /// a thread is working on the bin; just wait and pick other batches from a different island if available
		}


		/// parallel bin
		//const uint32 contact_batch_start = next_batch;
		//const uint32 contact_batch_end = VxMin(contact_batch_start + kBatchSize, constraint_count);

		///// CAS, if another thread beat us to the retrival of this bin 
		///// 
		//if (mNext.compare_exchange_weak(next_batch, contact_batch_end))

		//const uint32 constraint_count = bin_range.NumContactConstraint();
		//const uint32 contact_batch_start = bin_range.contactStart + next_batch;
		//const uint32 contact_batch_end = contact_batch_start + kBatchSize;
		////const uint32 contact_batch_end = VxMin(contact_batch_start + kBatchSize, constraint_count);

		///// CAS, if another thread beat us to the retrival of this bin 
		///// 
		//uint64 new_curr_bin_next = MakeCurrBinNext(curr_bin, next_batch + kBatchSize);
		//if (mCurrBinNext.compare_exchange_weak(curr_bin_next, new_curr_bin_next))
		//	//if (mNext.compare_exchange_weak(next_batch, next_batch + kBatchSize))
		//{
		//	o_contact_start = contact_batch_start;
		//	o_contact_end = contact_batch_end;

		//	//o_noncontact_start = bin_range.nonContactStart;
		//	//o_noncontact_end = bin_range.nonContactEnd;

		//	/// retrieved 
		//	return EStatus::RetrievedBatch;
		//}



		///we are going to grab a batch of contact and non contact
		/// but if one does not exist with pick 2 of the other 

		bool has_contact = (bin_range.contactStart + next_batch) < bin_range.contactEnd;
		bool has_noncontact = (bin_range.nonContactStart + next_batch) < bin_range.nonContactEnd;


		/// one as to be true
		const uint32 batch_size = (has_contact && has_noncontact) ? (kBatchSize / 2) : kBatchSize;

		uint32 contact_batch_start = 0;
		uint32 non_contact_batch_start = 0;
		uint32 contact_batch_end = 0;
		uint32 non_contact_batch_end = 0;

		if (has_contact)
		{
			contact_batch_start = bin_range.contactStart + next_batch;
			contact_batch_end = contact_batch_start + batch_size;
		}
		if (has_noncontact)
		{
			non_contact_batch_start = bin_range.nonContactStart + next_batch;
			non_contact_batch_end = non_contact_batch_start + batch_size;
		}

		/// CAS, if another thread beat us to the retrival of this bin 
		/// 
		uint64 new_curr_bin_next = MakeCurrBinNext(curr_bin, next_batch + batch_size);
		if (mCurrBinNext.compare_exchange_weak(curr_bin_next, new_curr_bin_next))
		{
			o_contact_start = contact_batch_start;
			o_contact_end = contact_batch_end;

			o_noncontact_start = non_contact_batch_start;
			o_noncontact_end = non_contact_batch_end;


			//first_iteration = (mIterations == 0);
			first_iteration = mIterations;

			/// retrieved 
			return EStatus::RetrievedBatch;
		}



		return EStatus::WaitingForBatches; /// a thread is working on the bin; just wait and pick other batches from a different island if available
	}

	void IslandCoordinator::Splitter::IslandSplitBins2::MarkConstraintBatchRangeComplete(uint32 process_constraint, uint32 velocity_iteration, bool& last_iteration, uint32 debug_bin)
	{
		/// 
		/// only a thread could mark a batch
		/// 
		//uint32 curr_bin_idx = mCurrBin.load(std::memory_order_relaxed);
		uint64 curr_bin_next = mCurrBinNext.load(std::memory_order_acquire);

		uint32 curr_bin_idx = GetCurrBin(curr_bin_next);
		uint32 next_batch = GetNextBatch(curr_bin_next);

		uint32 total_processed = mTotalBatchProcessed.fetch_add(process_constraint, std::memory_order_acq_rel) + process_constraint;
		//VX_ASSERT(total_processed <= mTotalBatchProcessed.load(std::memory_order_acquire));

		VX_ASSERT(curr_bin_idx == debug_bin, "Wrong bin!!!!!");


		//uint32 next_batch = mNext.load(std::memory_order_acquire);

		/// are we at the end of curr bin 
		BinConstraintsOffsetRange bin_range = mBins[curr_bin_idx];
		///next batch is numerial to to total
		//const bool next_end_of_bin = next_batch >= bin_range
		//const bool finished_curr_bin = total_processed >= next_batch &&
		//							next_batch >= bin_range.TotalConstraintCount();
		const bool finished_curr_bin = (total_processed == bin_range.TotalConstraintCount());


		VX_ASSERT(total_processed <= bin_range.TotalConstraintCount());

		last_iteration = (mIterations + 1 >= velocity_iteration);

		if (finished_curr_bin)
		{
			///at the end of current bin 
			/// and what defines last iteration 
			/// if this is non parallel bin or 
			/// curr bin is num acti e bin and no non paralllel bin
			/// 
			/// 


			//const bool finished_parallel_bin = curr_bin_idx + 1 >= mNumActiveBins;



			 

			const bool has_non_parallel_bin = mBins[kMaxBin].TotalConstraintCount() > 0;
			const uint32 last_bin_idx = (has_non_parallel_bin) ? kMaxBin : (mNumActiveBins - 1);
			const bool is_last_bin = (curr_bin_idx == last_bin_idx);

			//if ((curr_bin_idx >= mNumActiveBins && mBins[kMaxBin].TotalBatchCount() <= 0) || curr_bin_idx == kMaxBin)
			if (is_last_bin)
			{
				///this might determine next iteraction not completion
				if(last_iteration)
					mComplete.store(true, std::memory_order_release);
				else
				{
					uint32 new_bin = (mNumActiveBins > 0) ? 0 : kMaxBin;

					//mNext.store(0, std::memory_order_relaxed);
					mTotalBatchProcessed.store(0, std::memory_order_relaxed);

					//mCurrBin.store(new_bin, std::memory_order_release);
					uint64 new_bin_next = IslandSplitBins2::MakeCurrBinNext(new_bin, 0);
					mCurrBinNext.store(new_bin_next, std::memory_order_release);

					mIterations++;
					mFirstIteration.store(false, std::memory_order_release);
				}
			}
			else
			{

				//mNext.store(0, std::memory_order_relaxed);
				mTotalBatchProcessed.store(0, std::memory_order_relaxed);

				////if(curr_bin_idx == (mNumActiveBins - 1))

				//if (curr_bin_idx >= mNumActiveBins - 1) /// we are completed with parallel bins -> non parallel bin
				//{
				//	mCurrBin.store(kMaxBin, std::memory_order_release);
				//	VX_ASSERT(curr_bin_idx == (mNumActiveBins - 1));
				//}
				//else
				//	mCurrBin.fetch_add(1, std::memory_order_release);


				uint32 new_bin;
				if (curr_bin_idx >= mNumActiveBins - 1) /// we are completed with parallel bins -> non parallel bin
				{
					new_bin = kMaxBin;
					VX_ASSERT(curr_bin_idx == (mNumActiveBins - 1));
				}
				else
					new_bin = curr_bin_idx + 1;

				uint64 new_bin_next = IslandSplitBins2::MakeCurrBinNext(new_bin, 0);
				mCurrBinNext.store(new_bin_next, std::memory_order_release);

			}

			//uint32 new_bin = mCurrBin.load(std::memory_order_relaxed);
			uint64 new_bin_next = mCurrBinNext.load(std::memory_order_relaxed);
			uint32 new_bin = IslandSplitBins2::GetCurrBin(new_bin_next);
			VX_ASSERT(new_bin < mNumActiveBins || new_bin == kMaxBin);
			VX_ASSERT(mBins[new_bin].TotalConstraintCount() > 0);
		}
		else
		{
			//VX_ASSERT(curr_bin_idx == mCurrBin.load(std::memory_order_relaxed));
			//VX_ASSERT(total_processed <= mNext.load(std::memory_order_acquire));
		}
	}


	////////////////////////////////////////////////////////////////////////////////////////////////////////
	// SPLITTER
	////////////////////////////////////////////////////////////////////////////////////////////////////////


	IslandCoordinator::Splitter::EStatus IslandCoordinator::Splitter::NextConstactConstraintBatchRange(uint32& split_island_idx, uint32 island_count, IslandRange<uint32>& island_contact_range, IslandRange<uint32>& island_noncontact_range, int& batch_first_iteration, uint32& debug_bin, const uint32* sorted_island_indices)
	{
		bool complete = true;

		if (mStepLargeIslandCount <= 0)
			return EStatus::Complete;

		uint32 large_island_found = 0;

		//get first island with split
		for (uint32 i = 0; i < island_count; ++i)
		{
			uint32 island_idx = sorted_island_indices[i];
			if (large_island_found >= mStepLargeIslandCount)
				break;

			if (!mIslandIsLarge[island_idx])
				continue;

			large_island_found++;


			uint32 contact_start_offset;
			uint32 contact_end_offset;
			uint32 noncontact_start_offset;
			uint32 noncontact_end_offset;

			EStatus status = mIslandSplitBins2[island_idx].NextConstactConstraintBatchRange(contact_start_offset, contact_end_offset, noncontact_start_offset, noncontact_end_offset, batch_first_iteration, debug_bin);
			switch (status)
			{
			case EStatus::Complete:
				break;
			case EStatus::WaitingForBatches:
				complete = false;
				break;
			case EStatus::RetrievedBatch:

			{
				split_island_idx = island_idx;


#if VX_DEBUG_ISLAND_SPLITTER
				island_contact_range = IslandRange<uint32>(
					mConstraintIndices.data() + contact_start_offset,
					mConstraintIndices.data() + contact_end_offset
				);
				island_noncontact_range = IslandRange<uint32>(
					mConstraintIndices.data() + noncontact_start_offset,
					mConstraintIndices.data() + noncontact_end_offset
				);
#else
				island_contact_range = IslandRange<uint32>(
					mConstraintIndices + contact_start_offset,
					mConstraintIndices + contact_end_offset
				);

				island_noncontact_range = IslandRange<uint32>(
					mConstraintIndices + noncontact_start_offset,
					mConstraintIndices + noncontact_end_offset
				);
#endif // VX_DEBUG_SPLITTER

				///VX_ASSERT(batch_first_iteration == mIslandSplitBins2[island_idx].mIterations, (StackString<32>("1: ") << batch_first_iteration << "; 2: " << mIslandSplitBins2[island_idx].mIterations).Data());
				//batch_first_iteration = (mIslandSplitBins2[island_idx].mIterations == 0);
				uint32 constraint_offset_count = contact_end_offset - contact_start_offset;
				VX_ASSERT(constraint_offset_count == island_contact_range.Size());
				//VX_ASSERT(island_contact_range.Size() == 2);
				return EStatus::RetrievedBatch;
			}
			break;
			}
		}

		//failed to find a splitted island 
		return complete ? EStatus::Complete : EStatus::WaitingForBatches;
	}

	void IslandCoordinator::Splitter::MarkConstactConstraintBatchRangeComplete(uint32 island_idx, uint32 process_count, uint32 velocity_iteration, bool& last_iteration, uint32 debug_bin)
	{
		auto& island_split_bins = mIslandSplitBins2[island_idx];
		island_split_bins.MarkConstraintBatchRangeComplete(process_count, velocity_iteration, last_iteration, debug_bin);
	}

	void IslandCoordinator::Splitter::Prepare(uint32 active_count, const IslandCoordinator& island_coord, ScratchAllocator* scratch_allocator)
	{
		if (active_count <= 0 && island_coord.IslandCount() == 0)
			return;
#if VX_DEBUG_ISLAND_SPLITTER
		mBodyBinMasks.resize(active_count, 0);
		//mIslandIsLarge.clear();
		//if (mIslandIsLarge.size() < island_coord.IslandCount())
		mIslandIsLarge.resize(island_coord.IslandCount(), false);
		if(mIslandSplitBins2Size < island_coord.IslandCount())
		{
			delete[] mIslandSplitBins2;
			mIslandSplitBins2 = new IslandSplitBins2[island_coord.IslandCount()];
			mIslandSplitBins2Size = island_coord.IslandCount();
		}
#else
		mBodyBinMasks = reinterpret_cast<BinMask*>(scratch_allocator->Allocate(sizeof(BinMask) * active_count));
		mIslandIsLarge = reinterpret_cast<bool*>(scratch_allocator->Allocate(sizeof(bool) * island_coord.IslandCount()));
		std::memset(mIslandIsLarge, 0, island_coord.IslandCount() * sizeof(bool));
		mIslandSplitBins2Size = island_coord.IslandCount();
		mIslandSplitBins2 = reinterpret_cast<IslandSplitBins2*>(scratch_allocator->Allocate(sizeof(IslandSplitBins2) * island_coord.IslandCount()));
#endif // VX_DEBUG_SPLITTER




		mIslandCacheBins.resize(island_coord.IslandCount());
		mStepLargeIslandCount = 0;



		//mIslandSplitBins2.resize(island_coord.IslandCount());
		//for(auto& cache_bins : mIslandCacheBins)
		//	cache_bins.mConstraintIndicesBins

		///total constraints 
		uint32 total_constraint_count = 0;
		for (uint32 island_idx = 0; island_idx < island_coord.IslandCount(); ++island_idx)
		{
			IslandRange<uint32> contact_island = island_coord.ContactConstraintIndicesIslandRange(island_idx);
			IslandRange<uint32> non_contact_island = island_coord.IslandNonContactConstraintRowIndicesRange(island_idx); ///later its better to get actual constraint


			if (contact_island.Valid())
				total_constraint_count += contact_island.Size();

			if (non_contact_island.Valid())
				total_constraint_count += non_contact_island.Size();


			///quick reset 
			mIslandSplitBins2[island_idx].mNumActiveBins = 0;
			mIslandSplitBins2[island_idx].mTotalBatchProcessed ={ 0 };
			//mIslandSplitBins2[island_idx].mNext = { 0 };
			//mIslandSplitBins2[island_idx].mCurrBin = 0;
			mIslandSplitBins2[island_idx].mCurrBinNext = 0;

			mIslandSplitBins2[island_idx].mIterations = 0;

			mIslandSplitBins2[island_idx].mComplete = false;
			mIslandSplitBins2[island_idx].mFirstIteration = true;
			mIslandSplitBins2[island_idx].mBins[kMaxBin] = {};
			///uint32 
			//mIslandSplitBins2[island_idx].mTotalBatchProcessed = { 0 };
		}

#if VX_DEBUG_ISLAND_SPLITTER
		if (total_constraint_count > 0)
			mConstraintIndices.resize(total_constraint_count  +1);
#else
		mConstraintIndices = reinterpret_cast<uint32*>(scratch_allocator->Allocate(sizeof(uint32) * (total_constraint_count + 1)));
		mConstaintIndicesCount = total_constraint_count;
#endif // VX_DEBUG_SPLITTER



		mConstraintAllocatedTail = { 0 };
	}

	void IslandCoordinator::Splitter::ReleaseMemAllocation(ScratchAllocator* scratch_allocator, uint32 active_body_count, uint32 islands_count)
	{
		if (active_body_count <= 0 && islands_count == 0)
			return;
#if !VX_DEBUG_ISLAND_SPLITTER
		scratch_allocator->Free(mConstraintIndices, sizeof(uint32) * mConstaintIndicesCount + 1);
		scratch_allocator->Free(mIslandSplitBins2, sizeof(IslandSplitBins2) * islands_count);
		scratch_allocator->Free(mIslandIsLarge, sizeof(bool) * islands_count);
		scratch_allocator->Free(mBodyBinMasks, sizeof(BinMask) * active_body_count);

		mBodyBinMasks = nullptr;
		mIslandIsLarge = nullptr;
		mConstraintIndices = nullptr;
		mConstaintIndicesCount = 0;
#endif // !VX_DEBUG_SPLITTER
	}

	void IslandCoordinator::Splitter::SplitIsland(uint32 island_idx, const IslandCoordinator& island_coord, 
		const class ContactConstraintSolver* contact_coord, const class ConstraintSolver* constraint_solver, 
		class BodyManager* body_manager, ScratchAllocator* scratch_allocator)
	{
		IslandRange<uint32> contact_island = island_coord.ContactConstraintIndicesIslandRange(island_idx);
		IslandRange<uint32> non_contact_island = island_coord.IslandNonContactConstraintRowIndicesRange(island_idx); ///later its better to get actual constraint


		///quick hack refresh bins 
		IslandSplitBins& split_bins = GetIslandSplitBin(island_idx);
		for (uint32 i = 0; i < kMaxBin + 1; ++i)
		{
			split_bins.mConstraintIndicesBins[i].clear();
			split_bins.mNonConstraintIndicesBins[i].clear();
		}
		/// refresh bins regardlesss do not use old step bin if failed


		const uint32 contact_constraint_count = (contact_island.Valid()) ? contact_island.Size() : 0;
		const uint32 non_contact_constraint_count = (non_contact_island.Valid()) ? non_contact_island.Size() : 0;
		const uint32 total_constraint_count = contact_constraint_count + non_contact_constraint_count;

		if (total_constraint_count < kLargeIslandSpitThreshold)
		{
			mIslandIsLarge[island_idx] = false;
			return;
		}
		mIslandIsLarge[island_idx] = true;
		mStepLargeIslandCount++;

		//reset bins 
		//mBodyBinMasks.resize(body_manager->GetNumActiveBodies(), 0);

		IslandRange<BodyID> _island_bodies = island_coord.IslandBodyIDsRange(island_idx);
		for (const BodyID* body_id = _island_bodies.begin; body_id < _island_bodies.end; ++body_id)
			mBodyBinMasks[body_manager->GetBody(*body_id).GetIndexInActiveBodies()] = 0;



		/// go through island contact constraint
		for (const uint32* contact_idx = contact_island.begin; contact_idx < contact_island.end; ++contact_idx)
		{
			const auto& constraint = contact_coord->GetContactConstraint(*contact_idx);

			const Body& bodyA = constraint_solver->AttemptGetBody(body_manager, constraint->BodyA());
			const Body& bodyB = constraint_solver->AttemptGetBody(body_manager, constraint->BodyB());

			uint32 active_idxA = (bodyA.IsDynamic()) ? bodyA.GetIndexInActiveBodies() : Body::kInvalidActiveIdx;
			uint32 active_idxB = (bodyB.IsDynamic()) ? bodyB.GetIndexInActiveBodies() : Body::kInvalidActiveIdx;

			///fine for now 
			unsigned int split_idx = SplitParticipatingBodies(active_idxA, active_idxB);
			split_bins.mConstraintIndicesBins[split_idx].push_back(*contact_idx);
		}

		/// 
		/// go through island non contact constraint ///later its better to get actual constraint
		for (const uint32* non_contact_idx = non_contact_island.begin; non_contact_idx < non_contact_island.end; ++non_contact_idx)
		{
			const Body* bodyA = nullptr, *bodyB = nullptr;
			constraint_solver->AttemptGetLinear1DRowBodies(body_manager, *non_contact_idx, bodyA, bodyB);


			if (bodyA == nullptr || bodyB == nullptr)
				continue;

			///fine for now 
			unsigned int split_idx = SplitParticipatingBodies(bodyA->GetIndexInActiveBodies(), bodyB->GetIndexInActiveBodies());
			split_bins.mNonConstraintIndicesBins[split_idx].push_back(*non_contact_idx);
		}

		///Validate 
		for (uint32 i = 0; i < kMaxBin + 1; ++i)
		{
			for (auto& constraint_idx : split_bins.mConstraintIndicesBins[i])
				VX_ASSERT(constraint_idx < contact_coord->NumContactConstraints());

			//for (auto& constraint_idx : split_bins.mNonConstraintIndicesBins[i])
			//split_bins.mNonConstraintIndicesBins[i].clear();
		}

		for (const BodyID* body_id = _island_bodies.begin; body_id < _island_bodies.end; ++body_id)
		{
			Body& body = body_manager->GetBody(*body_id);
			body.mIslandConstraintGroupMask = mBodyBinMasks[body.GetIndexInActiveBodies()];
		}



		////allocation for this island
		uint32 constraint_write_spendle = mConstraintAllocatedTail;
		/// buffer is global not local to this island
		/// could make local so access/write with offset from buffer begin 
#if VX_DEBUG_ISLAND_SPLITTER
		uint32* constraint_indices_buffer = mConstraintIndices.data(); 
#else
		uint32* constraint_indices_buffer = mConstraintIndices; 
#endif // VX_DEBUG_SPLITTER

		mConstraintAllocatedTail += total_constraint_count;



		uint32* temp_non_parallel_buff = reinterpret_cast<uint32*>(scratch_allocator->Allocate(sizeof(uint32) * contact_constraint_count));
		uint32 temp_non_parallel_buff_count = 0;

		uint32* non_contact_non_parallel_buff = reinterpret_cast<uint32*>(scratch_allocator->Allocate(sizeof(uint32) * non_contact_constraint_count));
		uint32 non_contact_non_parallel_buff_count = 0;
		//uint32 contact_non_parallel_count = 0;
		

		//test bins 
		auto& curr_island_split_bins = mIslandSplitBins2[island_idx];
		// add contact constraint and non contact constraint to the constraint indices 
		///contact + non contact
		for (uint32 i = 0; i < kMaxBin + 1; ++i)
		{
			auto& contact_bin = split_bins.mConstraintIndicesBins[i];
			auto& non_contact_bin = split_bins.mNonConstraintIndicesBins[i];

			/// if less than batch size it non parallel bin 
			/// and if bin has remainder after multiples of batch size 
			/// remainder goes in non parallel bin 
			/// 
			if ((contact_bin.size() + non_contact_bin.size()) < kBatchSize || i == kMaxBin) //(i == kMaxBin) non parallel bin
			{
				for (const auto& idx : contact_bin)
					temp_non_parallel_buff[temp_non_parallel_buff_count++] = idx;

				for (const auto& idx : non_contact_bin)
					non_contact_non_parallel_buff[non_contact_non_parallel_buff_count++] = idx;

				continue;
			}

			/// multiples of batch size full count
			/// have to do contact and non contact seperatelty 
			/// contact and non contact range should not interleave
			BinConstraintsOffsetRange curr_bin_range;

			if(contact_bin.size() > 0)
			{
				/// batches 
				int full_batches = int(contact_bin.size()) / int(kBatchSize);
				int remainder = contact_bin.size() % kBatchSize;

				int total_batches = full_batches + (remainder > 0 ? 1 : 0);

				uint32 max_multiple = (contact_bin.size() / kBatchSize) * kBatchSize;

				if (full_batches > 0)
				{
					const uint32 bin_begin = constraint_write_spendle;

					for (uint32 j = 0; j < max_multiple; ++j)
						constraint_indices_buffer[bin_begin + j] = contact_bin[j];

					///end of wrrite 
					constraint_write_spendle = bin_begin + max_multiple;

					curr_bin_range.contactStart = bin_begin;
					curr_bin_range.contactEnd = constraint_write_spendle;
				}


				/// left over into non parallel bin
				if (remainder > 0)
				{
					VX_ASSERT(remainder == contact_bin.size() - max_multiple);

					///start from batch last 
					for (uint32 j = max_multiple; j < (max_multiple + remainder); ++j)
					{
						VX_ASSERT(j < contact_bin.size());
						temp_non_parallel_buff[temp_non_parallel_buff_count++] = contact_bin[j];
					}
				}
			}

			if(non_contact_bin.size() > 0)
			{
				/// batches 
				int full_batches = int(non_contact_bin.size()) / int(kBatchSize);
				int remainder = non_contact_bin.size() % kBatchSize;

				int total_batches = full_batches + (remainder > 0 ? 1 : 0);

				uint32 max_multiple = (non_contact_bin.size() / kBatchSize) * kBatchSize;

				if (full_batches > 0)
				{
					const uint32 _begin = constraint_write_spendle;

					for (uint32 j = 0; j < max_multiple; ++j)
						constraint_indices_buffer[_begin + j] = non_contact_bin[j];

					///end of wrrite 
					constraint_write_spendle = _begin + max_multiple;

					curr_bin_range.nonContactStart = _begin;
					curr_bin_range.nonContactEnd = constraint_write_spendle;

				}

				/// left over into non parallel bin
				if (remainder > 0)
				{
					VX_ASSERT(remainder == non_contact_bin.size() - max_multiple);

					///start from batch last 
					for (uint32 j = max_multiple; j < (max_multiple + remainder); ++j)
					{
						VX_ASSERT(j < non_contact_bin.size());
						non_contact_non_parallel_buff[non_contact_non_parallel_buff_count++] = non_contact_bin[j];
					}
				}
			}

			struct ValidateInfo
			{
				ValidateInfo(uint32 solver_body_idx, uint32 splitter_constraint_idx) : 
					data((solver_body_idx << 16) | splitter_constraint_idx){ }
				uint32 data;
			//private:
			};

			std::vector<ValidateInfo> mValidateInfo;

			//// validate that bodies does not appaer multiple 
			/// times within bins.
			auto& curr_island_split_bins = mIslandSplitBins2[island_idx];
			// add contact constraint and non contact constraint to the constraint indices 
			///contact + non contact
			for (uint32 i = 0; i < curr_island_split_bins.mNumActiveBins; ++i)
			{
				const BinConstraintsOffsetRange& bin = curr_island_split_bins.mBins[i];

				std::unordered_set<int> found_participating_bodies;

				///go through contact 
				for (uint32 contact_idx = bin.contactStart;
					contact_idx < bin.contactEnd; ++contact_idx)
				{
					///get constraint from global 
					const uint32 global_sim_constraint_idx = mConstraintIndices[contact_idx];

					const auto* contact_corrd = contact_coord->GetContactConstraint(global_sim_constraint_idx);

					if(constraint_solver->AttemptGetBody(body_manager, contact_corrd->BodyA()).IsDynamic())
						if (!found_participating_bodies.insert(contact_corrd->BodyA().Value()).second)
							mValidateInfo.push_back(ValidateInfo(contact_corrd->BodyA().Value(), contact_idx));

					if(constraint_solver->AttemptGetBody(body_manager, contact_corrd->BodyB()).IsDynamic())
						if (!found_participating_bodies.insert(contact_corrd->BodyB().Value()).second)
							mValidateInfo.push_back(ValidateInfo(contact_corrd->BodyB().Value(), contact_idx));
				}


				/// go through non contact
				//for (uint32 contact_idx = bin.nonContactStart;
				//	contact_idx < bin.nonContactEnd; ++contact_idx)
				//{
				//	///get constraint from global 
				//	const uint32 global_sim_constraint_idx = mConstraintIndices[contact_idx];

				//	const auto* contact_corrd = contact_coord->GetContactConstraint(global_sim_constraint_idx);

				//	if (!found_participating_bodies.insert(contact_corrd->BodyA().Value()).second)
				//		mValidateInfo.push_back(ValidateInfo(contact_corrd->BodyA().Value(), contact_idx));
				//	if (!found_participating_bodies.insert(contact_corrd->BodyB().Value()).second)
				//		mValidateInfo.push_back(ValidateInfo(contact_corrd->BodyB().Value(), contact_idx));
				//}



			}



			if (!mValidateInfo.empty())
			{
				VX_LOG_DEBUG("=====================================");
				VX_LOG_DEBUG("Failed to Validate Splitting of island: ", island_idx);

				for (const auto& info : mValidateInfo)
					VX_LOG_DEBUG("Solver Body: ", ((info.data & 0xffff0000) >> 16), ", Constraint: ", (info.data & 0x0000ffff));
				VX_LOG_DEBUG("=====================================");
			}

			
			//write range into bin
			if(curr_bin_range.TotalConstraintCount() > 0)
				curr_island_split_bins.mBins[curr_island_split_bins.mNumActiveBins++] = curr_bin_range;
		}


		///write non parallel bin / i.e last bin
		BinConstraintsOffsetRange non_parallel_bin_range;
		///start contact
		if (temp_non_parallel_buff_count > 0)
		{
			non_parallel_bin_range.contactStart = constraint_write_spendle;
			for (uint32* idx = temp_non_parallel_buff, *idx_end = temp_non_parallel_buff + temp_non_parallel_buff_count;
				idx < idx_end; ++idx)
				constraint_indices_buffer[constraint_write_spendle++] = (*idx);

			non_parallel_bin_range.contactEnd = constraint_write_spendle;
		}

		///start non contact
		if (non_contact_non_parallel_buff_count)
		{
			non_parallel_bin_range.nonContactStart = constraint_write_spendle;
			for (uint32* idx = non_contact_non_parallel_buff, *idx_end = non_contact_non_parallel_buff + non_contact_non_parallel_buff_count;
				idx < idx_end; ++idx)
				constraint_indices_buffer[constraint_write_spendle++] = (*idx);

			non_parallel_bin_range.nonContactEnd = constraint_write_spendle;
		}

		if (non_parallel_bin_range.TotalConstraintCount() > 0)
		{
			curr_island_split_bins.mBins[kMaxBin] = non_parallel_bin_range;

			if (curr_island_split_bins.mNumActiveBins <= 0) /// no parallel bins
				curr_island_split_bins.mCurrBinNext.store(IslandSplitBins2::MakeCurrBinNext(kMaxBin, 0), std::memory_order_relaxed);
		}
		VX_ASSERT((temp_non_parallel_buff_count + non_contact_non_parallel_buff_count) == curr_island_split_bins.mBins[kMaxBin].TotalConstraintCount());


		scratch_allocator->Free(non_contact_non_parallel_buff, sizeof(uint32)* non_contact_constraint_count);
		scratch_allocator->Free(temp_non_parallel_buff, sizeof(uint32)* contact_constraint_count);
	}


	unsigned int IslandCoordinator::Splitter::SplitParticipatingBodies(uint32 body_active_idxA, uint32 body_active_idxB)
	{

		if (body_active_idxB == Body::kInvalidActiveIdx)
		{
			BinMask& mask = mBodyBinMasks[body_active_idxA];

			uint32 split = ScanTrailingZeros(~uint32(mask));
			split = VxMin(split, kMaxBin);
			mask |= Bit32(split);//Bit8(split);
				return split;
		}
		else if (body_active_idxA == Body::kInvalidActiveIdx)
		{
			BinMask& mask = mBodyBinMasks[body_active_idxB];

			uint32 split = ScanTrailingZeros(~uint32(mask));
			split = VxMin(split, kMaxBin);
			mask |= Bit32(split);//Bit8(split);
				return split;
		}


		BinMask& maskA = mBodyBinMasks[body_active_idxA];
		BinMask& maskB = mBodyBinMasks[body_active_idxB];

		uint32 split = ScanTrailingZeros((~uint32(maskA) & ~uint32(maskB)));
		split = VxMin(split, kMaxBin);
		maskA |= Bit32(split);//Bit8(split);
		maskB |= Bit32(split);//Bit8(split);
		return split;

	}


	IslandCoordinator::IslandRange<uint32> IslandCoordinator::Splitter::ContactConstraintIndicesIslandRange(uint32 island_idx, uint32 bin) const
	{
		VX_ASSERT(island_idx < mIslandCacheBins.size());
		VX_ASSERT(bin < kMaxBin);

		const IslandSplitBins& split_bins = GetIslandSplitBin(island_idx);

		const std::vector<uint32>& constraint_indices_bins = split_bins.mConstraintIndicesBins[bin];

		if (constraint_indices_bins.empty())
			return IslandRange<uint32>(nullptr, nullptr);


		return IslandRange<uint32>(
			constraint_indices_bins.data(),
			constraint_indices_bins.data() + constraint_indices_bins.size()
		);
	}
} ///namespace vx 

