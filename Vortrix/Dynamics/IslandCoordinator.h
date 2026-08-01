#pragma once
#include "Vortrix/Core/Core.h"
#include "SolverBodyIndex.h"

#include "ConstraintSolver.h"

#include "Vortrix/Core/Atomics.h"
namespace vx {

	class IslandCoordinator
	{
	public:
		void Init(uint32 max_bodies)
		{
			mBodiesIdxs = new std::atomic<uint32>[max_bodies];
			//mIslandIdxs = new uint32[max_bodies];

			mIslandIdxs.resize(max_bodies);
		}

		void PrepareIslands(uint32 body_count)
		{
			for (uint32 i = 0; i < body_count; ++i)
				mBodiesIdxs[i] = i;

			islands.clear();
		}


		uint32 ComputeActiveBodyLowestIdx(uint32 idx)
		{
  	//		uint32 v = mBodiesIdxs[idx];

			//while (mBodiesIdxs[v] != v)
			//	v = mBodiesIdxs[v];

			//return v;


			uint32 v = idx;
			while (true)
			{
				uint32 body_links_to = mBodiesIdxs[v].load(std::memory_order_relaxed);
				if (body_links_to == v)
					break;
				v = body_links_to;
			}
			return v;
		}


		void LinkBodies(uint32 body_activeA, uint32 body_activeB)
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
						if (!mBodiesIdxs[l1].compare_exchange_weak(l1, l0, std::memory_order_relaxed))
							continue;
					}
					else
					{
						if (!mBodiesIdxs[l0].compare_exchange_weak(l0, l1, std::memory_order_relaxed))
							continue;
					}
				}

				uint32 lowest_link = VxMin(l0, l1);
				atomic::Min(mBodiesIdxs[body_activeA], lowest_link, std::memory_order_relaxed);
				atomic::Min(mBodiesIdxs[body_activeB], lowest_link, std::memory_order_relaxed);
				break;
			}


		}


		void LinkConstraint(uint32 constraint_idx, uint32 active_body_idx)
		{

		}





		void FinaliseIslands(ConstraintSolver& constraint_solver, BodyManager& body_manager)
		{
			///

			uint32 next_island_idx = 0;
			const uint32 active_bodies_count = body_manager.GetNumActiveBodies();

			///update island idxs 
			for (uint32 i = 0; i < active_bodies_count; ++i)
			{
				uint32 body_links = mBodiesIdxs[i];

				///links with self
				if (body_links == i)
					mIslandIdxs[i] = next_island_idx++;
				else
				{
					///not with self, Find 
					uint32 v = ComputeActiveBodyLowestIdx(body_links);
					if (v < i) //if behind, left side, already solve 
						mIslandIdxs[i] = mIslandIdxs[v];
					/// its ahead and need to be resolved
					else if(v == i) //another self link
					{
						mIslandIdxs[i] = next_island_idx++;
					}
					else
					{
						VX_LOG_INFO("Propably need to link with self");
					}
				}
				//if (body_links == i)
				//	mIslandIdxs[i] = next_island_idx++;
				//else
				//{
				//	//if (body_links < i) //if behind, left side, already solve 
				//	//	mIslandIdxs[i] = mIslandIdxs[body_links];
				//	//else
				//	//{
				//	//	VX_LOG_INFO("Propably need to link with self");

				//	//}
				//
				//	///not with self, Find 
				//	uint32 v = Find(body_links);
				//	if (v < i) //if behind, left side, already solve 
				//		mIslandIdxs[i] = mIslandIdxs[v];
				//	else
				//	{
				//		VX_LOG_INFO("Propably need to link with self");

				//	}
				//}


				//SolverBody& solver_body = constraint_solver.GetSolverBody(SolverBodyIndex(i));
				//body_manager.GetBody(active_bodies[i]).SetIslandIndex(mIslandIdxs[i]);
				body_manager.GetBody(body_manager.GetActiveBodyID(i)).SetIslandIndex(mIslandIdxs[i]);

				
			}





			if (islands.size() < next_island_idx)
				islands.resize(next_island_idx);


			for (auto& _island : islands)
				_island.bodyIds.clear();

			for (uint32 i = 0; i < active_bodies_count; ++i)
			{
				auto& body = body_manager.GetBody(body_manager.GetActiveBodyID(i));
				uint32 idx = body.GetIslandIndex();
				islands[idx].bodyIds.push_back(body.GetID());

			}


		}

		~IslandCoordinator()
		{
			delete[] mBodiesIdxs;
			//delete[] mIslandIdxs;
		}

	private:
		/// list would be invalid if not participanting or static 
		//uint32* mBodiesIdxs = nullptr;
		std::atomic<uint32>* mBodiesIdxs = nullptr;
		//uint32* mIslandIdxs = nullptr;
		std::vector<uint32> mIslandIdxs;

		uint32 mActiveCount;


	public:
		struct Island
		{
			std::vector<BodyID> bodyIds;
		};
		std::vector<Island> islands;
	};

} /// namespace vx 