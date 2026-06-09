#include "BVHBroadphase.h"
#include "Core/Profiler.h"
#include "Renderer/DebugGizmosRenderer.h"

#include "BroadphasePair.h"

#include "PhysicsWorld.h"

#include "Dynamics/Body/BodyManager.h"


namespace vx
{

	template<typename BoundType>
	void BVHBroadphase<BoundType>::Init(BodyManager* in_body_manager, const BroadphaseInitInfo& info)
	{
		VX_ASSERT(in_body_manager, "No bodies manager");
		mMaxBodies = in_body_manager->MaxBodies();

		//binary tree
		uint32 num_leaves_nodes = uint32(mMaxBodies);
		uint32 num_internal_nodes = num_leaves_nodes - 1;
		uint32 num_nodes = num_leaves_nodes + num_internal_nodes; // - 2N - 1

		mTree.SetLeafNodeMarginThreshold(mBoundThreshold);
		mTree.Init(in_body_manager, num_nodes);

		mMaxDirtyNodes = info.maxDirtyBodies;
		VX_ASSERT_WARN(mMaxDirtyNodes > 0, "Dynamics moving node wouldnt be update mMaxDirtyNode <= 0");
		mDirtyNodes = new BVH_NodeID[mMaxDirtyNodes];


		for (auto& body : in_body_manager->GetBodies())
			InsertBody(&body);


#if defined(VX_PROFILE_BROAD)
		VX_LOG_DEBUG("Verify nodes");
		for (auto& node : mTree.GetNodes())
		{
			if (node.IsLeaf())
				VX_LOG_DEBUG("leaf body: ", node.body->GetPosition(), " bounds {min: ", node.bounds.mMin, ", max: ", node.bounds.mMax, "}.");
			else
				VX_LOG_DEBUG("Interbal bounds{min: ", node.bounds.mMin, ", max: ", node.bounds.mMax, "}.");
		}

		mTree.DebugTree(mTree.GetRootID());
#endif // defined(VX_PROFILE_BROAD)
	}


	template<typename BoundType>
	void BVHBroadphase<BoundType>::InsertBody(Body* body)
	{
		//need to fix this later 
		//to get Bounds not a specific bound
		mTree.AddBody(body, body->GetAABBWorld());
	}

	template<typename BoundType>
	void BVHBroadphase<BoundType>::RemoveBody(const BodyID& id)
	{
		mTree.RemoveBody(id);
	}

	template<typename BoundType>
	void BVHBroadphase<BoundType>::ComputeCollidingPair(PhysicsStepContext& physics_ctx, BroadphasePair* io_pairs, uint32& io_count)
	{
		VX_PROFILE_FUNCTION();

		if (mTree.mNodes.size() <= 0)
			return;

		//Upodates 
		uint32 dirty_nodes_count = GatherDirtyNodes();

		// 
		// 1.check dirty up dirty
		mTree.UpdateDirtyNodes(mDirtyNodes, dirty_nodes_count);


		uint32 num_leaves_nodes = uint32(mMaxBodies);
		uint32 num_internal_nodes = num_leaves_nodes - 1;
		uint32 num_nodes = num_leaves_nodes + num_internal_nodes; // - 2N - 1


		//3. gather nodes leaf and reinsert
		bool rebuild = physics_ctx.forceBVHRebuild;

		if (!rebuild && physics_ctx.BVH_rebuild_SAH)
		{
			float ratio = mTree.ComputeDepthImbalance();
			rebuild = (ratio > physics_ctx.rebuildBVH_ImbalanceRatioTreshold);
		}
		//mTree.RebuildBruteforceInsertion();
		if(rebuild)
		{
			mTree.RebuildAllSAH(num_nodes);
			VX_LOG_INFO("Rebuilding BVH");
		}

		auto bvh_ctx = mTree.GetContext();
		//out_pairs.reserve(bvh_ctx.stats->leafCount);

		mTree.ComputeCollidingPairs(physics_ctx, io_pairs, io_count);

		mLastStep = PhysicsWorld::GetCurrentSimStep();
	}

	template<typename BoundType>
	void BVHBroadphase<BoundType>::DebugDraw(DebugGizmosRenderer* debug_renderer, const DrawSettings& settings)
	{
		if(vx::Contains(settings.braodphaseFlags, EBroadphaseDrawFlag::LeafNodes) &&
			vx::Contains(settings.braodphaseFlags, EBroadphaseDrawFlag::InternalNodes))
		{
			float thick[2] = { debug_renderer->GetLineWidth(), debug_renderer->GetLineWidth() * 1.75f };
			for (const auto& node : mTree.GetNodes())
			{
				debug_renderer->SetLineWidth(thick[node.IsLeaf()]);
				debug_renderer->DrawAABB(node.bounds.mMin,
					node.bounds.mMax, vx::Colour(0.0f, 0.0f, 1.0f));
			}
			debug_renderer->SetLineWidth(thick[0]);
		}
		else if (Contains(settings.braodphaseFlags, EBroadphaseDrawFlag::LeafNodes))
		{
			const auto& leaf_node_ids = mTree.GetLeafNodeIDs();
			for (int i = 0; i < mTree.GetLeafNodeCount(); ++i)
			{
				const auto& node = *mTree.GetNode(leaf_node_ids[i]);

				debug_renderer->DrawAABB(node.bounds.mMin,
					node.bounds.mMax, vx::Colour(0.0f, 0.0f, 1.0f));
			}


			debug_renderer->DrawAABB(mTree.mLeafNodesBound.mMin,
				mTree.mLeafNodesBound.mMax, vx::Colour(0.0f, 0.0f, 1.0f));
		}
	}

	template<typename BoundType>
	uint32 BVHBroadphase<BoundType>::GatherDirtyNodes()
	{
		///collect 
		uint32 dirty_node_count = 0;

		//quicxk 
		mTree.mLeafNodesBound.Reset();


		VX_ASSERT(mTree.mNodes.size() > 0, "No nodes !!!");

		const auto& n = *mTree.GetNode(mTree.GetLeafNodeIDs()[0]);
		mTree.mLeafNodesBound.Merge(n.bounds);

		const auto& leaf_node_ids = mTree.GetLeafNodeIDs();
		for (int i = 0; i < mTree.GetLeafNodeCount(); ++i)
		{
			const auto& n = *mTree.GetNode(leaf_node_ids[i]);

			VX_ASSERT(!n.IsInvalid(), "Node is invalid");

			bool dirty = false;
			bool moved = n.body->GetTransformedState().MovedSince(mLastStep);

			switch (mTree.GetNodeUpdateState())
			{
			case UpdateNode::Normal:
				dirty = (n.body->IsAwake() && moved
					&& !n.bounds.Contains(n.body->GetAABBWorld()));
				break;
			case UpdateNode::UpdateIfMoved:
				dirty = (n.body->IsAwake() && moved);
				break;
			case UpdateNode::AlwaysUpdate:
			case UpdateNode::UpdateOnMarginChanged:
				dirty = true;
				break;
			}

			if (dirty)
				mDirtyNodes[dirty_node_count++] = n.id;

			if (dirty_node_count >= mMaxDirtyNodes) break;


			//quick hack for now 
						//quiclk 
			mTree.mLeafNodesBound.Merge(n.bounds);
		}

		
		VX_ASSERT_WARN(dirty_node_count <= mMaxDirtyNodes, "node tree growing over max dirty node!!");

		return dirty_node_count;
	}

	template<typename BoundType>
	void BVHBroadphase<BoundType>::CastRay(const RayCast& ray_cast, WorldRayCastQuery& world_ray_ctx) const
	{
		if (mTree.mNodes.size() <= 0)
			return;

		mTree.CastRay(ray_cast, world_ray_ctx);
	}


	//template class BVHTree<AABB>::Node;
	template class BVHTree<AABB>;
	template class BVHBroadphase<AABB>;
}

