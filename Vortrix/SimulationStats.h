#pragma once
#include "Core/Core.h"
#include "SimulationContexts.h"

#include "Dynamics/IslandCoordinator.h"

namespace vx {
	struct SolverWorkloadStats
	{
		uint32 islandCount = 0;
		uint32 totalConstraints = 0;

		uint32 maxIslandConstraint = 0;
		uint32 normalIslandCount = 0;
		uint32 largeIslandCount = 0;

		double totalJacobianRows = 0.0;

		bool splitLargeIsland = true;
		int largeIslandThreshold = 2;
	};


	static SolverWorkloadStats CollectSolverWorkloadStats(const PhysicsStepContext& ctx)
	{
		SolverWorkloadStats stats;

		const auto* island_coord = ctx.mIslandCoordinator;

		if (!island_coord)return{};

		stats.islandCount = island_coord->IslandCount();


		const bool split_large_island = ctx.mSettings->splitLargeIsland;

		for (uint32 i = 0; i < stats.islandCount; ++i)
		{
			const auto& noncontact_range = 
				island_coord->IslandNonContactConstraintRowIndicesRange(i);

			const auto& contact_range = 
				island_coord->ContactConstraintIndicesIslandRange(i);

			uint32 noncontact_count = 0;
			uint32 contact_count = 0;

			if (noncontact_range.Valid())
				noncontact_count = noncontact_range.Size();
			if (contact_range.Valid())
				contact_count = contact_range.Size();

			const uint32 constraint_count = noncontact_count + contact_count;



			///stats 
			stats.totalConstraints += constraint_count;
			stats.maxIslandConstraint = VxMax(stats.maxIslandConstraint, constraint_count);

			if (split_large_island && island_coord->GetSplitter().IsIslandLarge(i))
				++stats.largeIslandCount;
			else
				++stats.normalIslandCount;
		}

		return stats;
	}







	struct WorkerLoadStats
	{
		uint32 workerCount = 0;
		uint64 totalJacobianRows = 0;

		uint32 maxWorkerRows = 0;
		uint32 minWorkerRows = 0;

		double meanWorkerRows = 0.0;

		double loadBalanceEff = 0.0;
		double loadImbalance = 0.0;
	};

}