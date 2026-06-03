#pragma once

#include <Vortrix.h>
#include "CollisionDispatcher.h"
#include "NarrowphaseQueryStat.h"//


namespace vx {

	class BodyManager;
	class NarrowphaseQuery
	{
	public:

		NarrowphaseQuery();
		void Init(BodyManager* in_body_manager);

		const CollisionResolutionStat& Stats() const { return mStats; }

		void ProcessPairs(const std::vector<struct BroadphasePair>& pairs, std::vector<ContactManifold>& out_manifolds, class ContactConstraintSolver& contact_solver, const struct CollisionContext& ctx);

	private:
		CollisionDispatcher mDispatcher;

		CollisionResolutionStat mStats;

		//used for bodies simulation stat update
		BodyManager* mBodyManager = nullptr;
	};
}