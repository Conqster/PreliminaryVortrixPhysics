#pragma once

#include <Vortrix/Vortrix.h>
#include "CollisionDispatcher.h"
#include "NarrowphaseQueryStat.h"//


namespace vx {

	class BodyManager;
	struct SimStep;

	class NarrowphaseQuery
	{
	public:

		NarrowphaseQuery();

		const CollisionResolutionStat& Stats() const { return mStats; }

		bool ProcessPairAndTrySetupContactConstraint(const Body* a, const Body* b, ContactConstraintSolver& contact_solver, SimStep* io_step);
		void ProcessPairs(struct BroadphasePair* in_pairs, class ContactConstraintSolver& contact_solver, SimStep* io_step);

	private:
		CollisionDispatcher mDispatcher;
		CollisionResolutionStat mStats;
	};
}


/// note (what is not thread safe): 
/// ManifoldMapEntry new_manifold_entry = write_manifold_cache.Create(key, CachedManifold(manifold.a->ID(), manifold.b->ID(), num_contact_pts));
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