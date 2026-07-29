#pragma once

#include <Vortrix/Vortrix.h>
#include "CollisionDispatcher.h"
#include "NarrowphaseQueryStat.h"//


namespace vx {

	class BodyManager;
	struct CollisionContext;

	class NarrowphaseQuery
	{
	public:

		NarrowphaseQuery();
		void Init(BodyManager* in_body_manager);

		const CollisionResolutionStat& Stats() const { return mStats; }

		void ProcessPairAndTrySetupContactConstraint(const Body* a, const Body* b, ContactConstraintSolver& contact_solver, const CollisionContext& ctx);
		void ProcessPairs(struct BroadphasePair* in_pairs, std::vector<ContactManifold>& out_manifolds, class ContactConstraintSolver& contact_solver, const CollisionContext& ctx);

	private:
		CollisionDispatcher mDispatcher;

		CollisionResolutionStat mStats;

		//used for bodies simulation stat update
		BodyManager* mBodyManager = nullptr;
	};
}


/// note (what is not thread safe): 
/// ManifoldMapEntry new_manifold_entry = write_manifold_cache.Create(key, CachedManifold(manifold.a->GetID(), manifold.b->GetID(), num_contact_pts));
/// 
/// ManifoldMap& read_manifold_cache = mManifoldCache[mManifoldWriteCache ^ 1]; (partial)
/// 
/// THIS IS A POTENTIAL RACE CONDITION
/// - 2 or more thread, could pick the same idx 
/// - then use, the slot would be overriden
/// - incremental of count, if wrong would cause race conidition
///		//allocate mem
///		uint32 idx = mNumConstraints;
///		VX_ASSERT_WARN_VOID(idx < mMaxConstraints, "Max frame contact constraint attianed returning");
///		ContactConstraint& constraint = mConstraints[idx];
///		mNumConstraints++;
/// 
/// 
/// potential fix; 
/// - each threads/workers owns a contact constraint, then merge at the end
/// - simple lockfree, CAS for allocating constraints, but manifold map would require heavy code work