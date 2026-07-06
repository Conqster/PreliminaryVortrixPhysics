#include "NarrowphaseQuery.h"

#include "Vortrix/Collision/Broadphase/BroadphasePair.h"

#include "CollisionAlgorithms.h"
#include "Vortrix/Core/Profiler.h"
#include "Vortrix/Dynamics/Body/EBodyDebugFlags.h"

#include "Vortrix/Dynamics/Body/Body.h"
#include "Vortrix/Dynamics/ConstraintSolver/ContactConstraintSolver.h"

#include "Vortrix/Dynamics/Body/BodyManager.h"
#include "Vortrix/Dynamics/Body/BodySimStats.h"

#include "Vortrix/SimulationContexts.h"

namespace vx {

	NarrowphaseQuery::NarrowphaseQuery()
	{
		mDispatcher.Register(EShapeType::Sphere, EShapeType::Sphere, &Narrowphase::SphereVsSphere);

		mDispatcher.Register(EShapeType::Sphere, EShapeType::Plane, &Narrowphase::SphereVsPlane);
		mDispatcher.Register(EShapeType::Plane, EShapeType::Sphere, &CollisionDispatcher::SwappedRef<Narrowphase::SphereVsPlane>);

		mDispatcher.Register(EShapeType::Box, EShapeType::Plane, &Narrowphase::BoxVsPlane);
		mDispatcher.Register(EShapeType::Plane, EShapeType::Box, &CollisionDispatcher::SwappedRef<Narrowphase::BoxVsPlane>);

		mDispatcher.Register(EShapeType::Box, EShapeType::Sphere, &Narrowphase::BoxVsSphere);
		mDispatcher.Register(EShapeType::Sphere, EShapeType::Box, &CollisionDispatcher::SwappedRef<Narrowphase::BoxVsSphere>);

		mDispatcher.Register(EShapeType::Box, EShapeType::Box, &Narrowphase::BoxVsBox);

		mDispatcher.Register(EShapeType::Capsule, EShapeType::Plane, &Narrowphase::CapsuleVsPlane);
		mDispatcher.Register(EShapeType::Plane, EShapeType::Capsule, &CollisionDispatcher::SwappedRef<Narrowphase::CapsuleVsPlane>);

		mDispatcher.Register(EShapeType::Capsule, EShapeType::Sphere, &Narrowphase::CapsuleVsSphere);
		mDispatcher.Register(EShapeType::Sphere, EShapeType::Capsule, &CollisionDispatcher::SwappedRef<Narrowphase::CapsuleVsSphere>);

		mDispatcher.Register(EShapeType::Capsule, EShapeType::Capsule, &Narrowphase::CapsuleVsCapsule);

		mDispatcher.Register(EShapeType::Box, EShapeType::Capsule, &Narrowphase::BoxVsCapsule);
		mDispatcher.Register(EShapeType::Capsule, EShapeType::Box, &CollisionDispatcher::SwappedRef<Narrowphase::BoxVsCapsule>);
	}

	void NarrowphaseQuery::Init(BodyManager* in_body_manager)
	{
		mBodyManager = in_body_manager;
	}

	void NarrowphaseQuery::ProcessPairs(BroadphasePair* in_pairs, std::vector<ContactManifold>& out_manifolds, ContactConstraintSolver& contact_solver, const CollisionContext& ctx)
	{
		VX_PROFILE_FUNCTION();

#define NEED_REMOVE 0
#if NEED_REMOVE
		out_manifolds.clear();
		out_manifolds.reserve(pairs.size());
#endif // NEED_REMOVE

		mStats.numPairReceived = ctx.broadphasePairCount;

		for (BroadphasePair* bp = in_pairs, *bp_end = in_pairs+ctx.broadphasePairCount; 
			bp < bp_end; ++bp)
		{
			Body* a = (*bp).a;
			Body* b = (*bp).b;

			BodySimStats& body_a_stat = mBodyManager->GetBodySimStats(*a); 
			BodySimStats& body_b_stat = mBodyManager->GetBodySimStats(*b); 

			body_a_stat.phase |= EBodySimphaseFlags::InNarrowphase;
			body_b_stat.phase |= EBodySimphaseFlags::InNarrowphase;

			if (a != nullptr && b != nullptr)
			{
				//Shape* shape_a = a->GetShape();

				const auto& shape_a = a->GetShape();
				const auto& shape_b = b->GetShape();
				const auto& collision_fn = mDispatcher.Get(shape_a->GetType(), shape_b->GetType());

				ContactManifold manifold{ (*bp).a, (*bp).b };
				if (collision_fn(shape_a, a->GetPosition(), a->GetOrientation(),
					shape_b, b->GetPosition(), b->GetOrientation(),
					manifold))
				{
					//use opptunity to add to contact constaint
					contact_solver.SetupContactConstraint(manifold, ctx);

					//later move in collision fn when check if static
					body_a_stat.phase|= EBodySimphaseFlags::IsColliding;
					body_b_stat.phase|= EBodySimphaseFlags::IsColliding;

					BodySimStats* colliding_static_stat = (b->IsStatic()) ? &body_a_stat : (a->IsStatic()) ? &body_b_stat : nullptr;

					(colliding_static_stat != nullptr) ? (colliding_static_stat->phase |= EBodySimphaseFlags::IsTouchingStatic) : EBodySimphaseFlags::None;
#if NEED_REMOVE
					out_manifolds.push_back(manifold);
#endif // NEED_REMOVE
				}
			}
		}

		mStats.numContactPair = out_manifolds.size();
		mStats.maxAttainedContactPair = VxMax(mStats.maxAttainedContactPair, mStats.numContactPair);


		//add contact constraint
	}

}