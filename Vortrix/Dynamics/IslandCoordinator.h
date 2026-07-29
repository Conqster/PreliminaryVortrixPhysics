#pragma once
#include "Vortrix/Core/Core.h"
#include "SolverBodyIndex.h"

#include "ConstraintSolver.h"
namespace vx {

	class IslandCoordinator
	{
	public:
		void Init(uint32 max_bodies)
		{
			mBodiesIdxs = new uint32[max_bodies];
			//mIslandIdxs = new uint32[max_bodies];

			mIslandIdxs.resize(max_bodies);
		}

		void PrepareIslands(uint32 body_count)
		{
			for (uint32 i = 0; i < body_count; ++i)
				mBodiesIdxs[i] = i;
		}


		uint32 Find(uint32 idx)
		{
			uint32 v = mBodiesIdxs[idx];

			while (mBodiesIdxs[v] != v)
				v = mBodiesIdxs[v];

			return v;
		}

		void UnionFind(uint32 idx0, uint32 idx1)
		{
			uint32 l0 = Find(idx0);
			uint32 l1 = Find(idx1);

			if (l0 > l1)
				std::swap(l0, l1);

			mBodiesIdxs[l1] = l0;

			mBodiesIdxs[idx0] = l0;
			mBodiesIdxs[idx1] = l0;
		}


		void LinkBodies(uint32 body_activeA, uint32 body_activeB)
		{
			UnionFind(body_activeA, body_activeB);
		}

		void FinaliseIslands(ConstraintSolver& constraint_solver, BodyID* active_bodies, uint32 count, BodyManager& body_manager)
		{


			uint32 next_island_idx = 0;

			///update island idxs 
			for (uint32 i = 0; i < count; ++i)
			{
				uint32 body_links = mBodiesIdxs[i];

				///links with self
				if (body_links == i)
					mIslandIdxs[i] = next_island_idx++;
				else
				{
					///not with self, Find 
					uint32 v = Find(body_links);
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
				body_manager.GetBody(active_bodies[i]).islandIdx = mIslandIdxs[i];
			}





			if (islands.size() < next_island_idx)
				islands.resize(next_island_idx);


			for (auto& _island : islands)
				_island.idx.clear();

			for (uint32 i = 0; i < count; ++i)
			{
				auto& body = body_manager.GetBody(active_bodies[i]);
				uint32 idx = body.islandIdx;
				islands[idx].idx.push_back(body.GetID().Idx());

			}


		}

		~IslandCoordinator()
		{
			delete[] mBodiesIdxs;
			//delete[] mIslandIdxs;
		}

	private:
		/// list would be invalid if not participanting or static 
		uint32* mBodiesIdxs = nullptr;
		//uint32* mIslandIdxs = nullptr;
		std::vector<uint32> mIslandIdxs;

		uint32 mActiveCount;


	public:
		struct Island
		{
			std::vector<uint32> idx;
		};
		std::vector<Island> islands;
	};

} /// namespace vx 