#pragma once

#include "Vortrix/Vortrix.h"
#include "Vortrix/Collision/Shapes/Shape.h"
#include "Vortrix/Geometry/AABB.h"

#include <stack>
#include <unordered_set>

#include "Broadphase.h"

#include "BVHTree.h"
#include "Vortrix/Dynamics/Body/Body.h"

namespace vx
{
	class Body;
	class BodyManager;
	struct BroadphasePair;

	//It might be btter to have an interface/abstract form BoundType
	//but i want to not complicate AABB etc geometry
	//because i might decisde to use it for graphics etc
	//template<typename BoundType>
	//concept BoundVolumeConcept = requires(const BoundType a, const BoundType b, Vec3 v) {
	//	{ a.GetCenter() } -> std::same_as<Vec3>;
	//};
	template<typename BoundType>
	class BVHBroadphase final : public Broadphase
	{
	public:
		using TreeType = BVHTree<BoundType>;
		using NodeType = typename TreeType::Node;
		using BVH_NodeID = typename TreeType::NodeID;


		void Init(BodyManager* in_body_manager, const BroadphaseInitInfo& info) override;

		~BVHBroadphase()
		{
			delete[] mDirtyNodes;
		}

		void InsertBody(Body* body) override;
		void RemoveBody(const BodyID& id) override;

		BVHTree<AABB>::BVHContext GetTreeStat() { return mTree.GetContext(); }
		void ComputeCollidingPair(struct PhysicsStepContext& physics_ctx, BroadphasePair* io_pairs, uint32& io_count) override;

		void SetBoundThreshold(float v) override
		{
			Broadphase::SetBoundThreshold(v);
			mTree.SetLeafNodeMarginThreshold(v);
		}

		void DebugDraw(DebugGizmosRenderer* debug_renderer, const DrawSettings& settings) override;

		uint32 GatherDirtyNodes();

		virtual void CastRay(const RayCast& ray_cast, WorldRayCastQuery& world_ray_ctx) const override;

#if defined(VX_PROFILE_BROAD)
		EBroadphaseType Type() const override { return EBroadphaseType::BVH; }
#endif // defined(VX_PROFILE_BROAD)
	private:
		TreeType mTree = {}; //could be layers ike dynamic, static
		size_t mMaxBodies = 128;

		//std::vector<BroadphasePair> mPairs;
		//std::vector<Body>* mTargetBodies;
		BVH_NodeID* mDirtyNodes = nullptr;
		size_t mMaxDirtyNodes = 128;
	};


}