#include "BVHTree.h"

#include "BroadphasePair.h"
#include "Broadphase.h"

#include "Geometry/AABB.h"
#include "Dynamics/Body/Body.h"

#include "Core/Profiler.h"

#include <algorithm>

#include "Dynamics/Body/BodyManager.h"
#include "Dynamics/Body/EBodyDebugFlags.h"

#include "Core/StackString.h"


namespace vx
{
	template<typename BoundType>
	void BVHTree<BoundType>::Init(BodyManager* in_body_manager, uint node_limit)
	{
		mNodes.reserve(node_limit);
		mStats.nodeCount = 0;
		mRootID = kInvalidNode;
		VX_LOG_DEBUG("size of Node<AABB>", sizeof(BVHTree<BoundType>::Node));

		mBodyManager = in_body_manager;

		mLeafNodeCount = 0;
		mLeafNodeIDs = new NodeID[mBodyManager->MaxBodies()];
	}
	template<typename BoundType>
	inline void BVHTree<BoundType>::AddBody(Body* body, const BoundType& bounds)
	{
		//Create new node

		//try adding new node
		//Insert(GetContext(), new Node());


		//Insert node
		if (mRootID == kInvalidNode)
		{
			mRootID = EmplaceNode(kInvalidNode, bounds, body);
			//mRoot->Refit(mLeafNodeMargin);
			mStats.nodeCount += 1;
			mStats.leafCount += 1;
		
			bool tracking_leaf = TrackLeafNode(mRootID);
			VX_ASSERT_WARN(tracking_leaf, "Unable to track body might be a internal node");
		}
		else
		{
			//create new leaf node 
			mStats.nodeCount += 1;
			mStats.leafCount += 1;
			NodeID new_node_id = EmplaceNode(kInvalidNode, bounds, body);
			//new_node->Refit(mLeafNodeMargin);
			//recuresive insert into tree
			InsertNode(GetContext(), new_node_id, mRootID);
		}
	}

	template<typename BoundType>
	inline void BVHTree<BoundType>::InsertNode(const BVHContext& bvh_context, NodeID new_node_id, NodeID tranverse_node_id)
	{
		//ensure this is a valid node 
		VX_ASSERT((new_node_id != kInvalidNode && tranverse_node_id != kInvalidNode), "Invaild nodes");

		Node& new_node = mNodes[new_node_id];
		Node& parent = mNodes[tranverse_node_id];
		new_node.depth++;


		/// Surface Area Heuristic (SAH)
		auto compute_surface_area_growth = [](const BoundType& a, const BoundType& b) {
			return a.Merged(b).GetSurfaceArea() - a.GetSurfaceArea();
			};

		//else transvere a branch
		if (parent.IsLeaf())
		{
			//crete new parent node to take tranerse node place
			//when leaf recomputed
			NodeID replace_node_id = EmplaceNode(tranverse_node_id, parent.bounds, parent.body);
			Node& replace_node = mNodes[replace_node_id];
			//replace_node->Refit(mLeafNodeMargin);
			mStats.nodeCount += 1;
			//replace node get the transverrse node depth and one down
			replace_node.depth = parent.depth + 1;

			//uodate tranverse node
			parent.children[0] = replace_node_id;
			parent.children[1] = new_node_id;

			parent.body = nullptr;
			replace_node.parent = tranverse_node_id; //<-- should already be set when emplaing node
			new_node.parent = tranverse_node_id;

			//need to replace the leaf idx as well
			//replace_node.leafNodeIdx = parent.leafNodeIdx;
			bool tracking_leaf = TrackLeafNode(replace_node_id);
			tracking_leaf |= TrackLeafNode(new_node_id);
			VX_ASSERT_WARN(tracking_leaf, "Unable to track body might be a internal node");
			RemoveTrackingNode(tranverse_node_id);

			float left_growth_cost = compute_surface_area_growth(replace_node.bounds, new_node.bounds);
			float right_growth_cost = compute_surface_area_growth(new_node.bounds, replace_node.bounds);

			if (right_growth_cost < left_growth_cost)
				std::swap(parent.children[0], parent.children[1]);

			//RecomputeBounds()
			//mark as dirty 
			//during update transverse linear buffer 
			//in reverse with garantee child node are updated before parent
			//maybe recurese recompute bounds better 
			//actually no, becase that means a parent woulkd have to 
			//recompuet bounds mutilple time. 
			//but again that makes a better sturture because.
			//if object are added consecutively without 
			//recompute insertion would be wrong
			//tranverse_node->RecomputeBounds(mLeafNodeMargin);
			//new_node->RecomputeBounds(mLeafNodeMargin); //<-- in most case this has already be computed on construction same with tranverse_node
			//and also recompute bounds is if children node as change 
			// i.e new_node & transverse are leaf so Refit is more suited so 
			// leaf could adjust with margin around data/body 
			// 	
			//void Refit(float body_bounds_margin = 0.2f) {//20cm
			//	if (IsLeaf())
			//		//update AABB from the body
			//		if (body)
			//		{
			//			bounds = body->GetAABBWorld();
			//			bounds.Grow(body_bounds_margin);
			//		}
			//		else
			//			bounds = children[0]->bounds.Merged(children[1]->bounds);
			//	dirty = false;
			//	if (parent)
			//		parent->Refit();}
			//maybe just call RecomputeBounds instead of parent refit as parent is not leaf
			//replace_node->Refit(mLeafNodeMargin);//<-- refit is already called when creating new node only transverse is dirty
			RecomputeBounds(tranverse_node_id);//<-- refit is already called when creating new node only transverse is dirty
		}
		else
		{
			VX_ASSERT((parent.children[0] != kInvalidNode && parent.children[1] != kInvalidNode), "Invaild node children");
			Node& child0 = mNodes[parent.children[0]];
			Node& child1 = mNodes[parent.children[1]];

			/// otherwise, work out which child gets to keep/transvasre 
			/// towards the insterd body
			/// target node that requires less growth
			float left_growth = compute_surface_area_growth(child0.bounds, new_node.bounds);
			float right_growth = compute_surface_area_growth(child1.bounds, new_node.bounds);

			NodeID tranversal_branch = (left_growth < right_growth) ? parent.children[0] : parent.children[1];
			//recursive transversal 
			InsertNode(bvh_context, new_node_id, tranversal_branch);
			//recursive stack 
			// actually recompute only when a leaf as been added
			// because that means this would recompute twice
			// as Recompute is bottom up (recursic)
			//RecomputeBounds()
		}
	}
	//template<typename BoundType>
	//void BVHTree<BoundType>::MarkNodeDirty(NodeID node_id)
	//{
	//	if (node_id == kInvalidNode) return;
	//	Node& node = mNodes[node_id];
	//	if (node.dirty) return;

	//	node.dirty = true;
	//	if (node.parent != kInvalidNode)
	//		MarkNodeDirty(node.parent);
	//	//later support a linear backward loop from 
	//	//idx point
	//}
	template<typename BoundType>
	void BVHTree<BoundType>::RefitNode(NodeID node_id, float bounds_margin)
	{
		VX_PROFILE_FUNCTION();
		if (node_id == kInvalidNode) return;
		NodeID curr_id = node_id;
		while (curr_id != kInvalidNode)
		{
			Node& n = mNodes[curr_id];
			//if (!n.dirty)
			//{
			//	curr_id = n.parent;
			//	continue;
			//}

			if (n.IsLeaf())
			{
				n.bounds = n.body->GetAABBWorld();
				n.bounds.Grow(bounds_margin);
			}
			else
			{
				Node& c0 = mNodes[n.children[0]];
				Node& c1 = mNodes[n.children[1]];
				n.bounds = c0.bounds.Merged(c1.bounds);
			}
			//n.dirty = false;
			curr_id = n.parent;
		}
		//parent->Refit();
		//maybe just call RecomputeBounds instead of parent refit as parent is not leaf

	}
	template<typename BoundType>
	void BVHTree<BoundType>::RecomputeBounds(const NodeID& node_id)
	{
		if (node_id == kInvalidNode) return;
		Node& node = mNodes[node_id];
		if (node.IsLeaf()) return;

		Node& c0 = mNodes[node.children[0]];
		Node& c1 = mNodes[node.children[1]];
		node.bounds = c0.bounds.Merged(c1.bounds);

		//node.dirty = false;
		if (node.parent != kInvalidNode)
			RecomputeBounds(node.parent);

	}
	template<typename BoundType>
	void BVHTree<BoundType>::UpdateDirtyNodes(const NodeID* dirty_leave_nodes, uint32 dirty_node_count/*uint32 last_broadphase_step*/)
	{
		VX_PROFILE_FUNCTION();
#if defined(VX_PROFILE_BROAD)
		mStats.dirty_leaf_nodes = 0;
		mStats.dirty_nodes = dirty_node_count;
		mStats.max_dirty_nodes = VxMax(mStats.max_dirty_nodes, dirty_node_count);
#endif // defined(VX_PROFILE_BROAD)


		//mark visited node

		//NEW 
		//static uint32 visited[16383];
		//uint32 _max_vis = VxMin(16383, int(mNodes.size()));
		////std::memset(visited, 0, mNodes.size() * sizeof(uint32));
		//std::memset(visited, 0, _max_vis * sizeof(uint32));
		
		static std::vector<uint32> visited(mNodes.capacity(), 1);
		visited.reserve(mNodes.size());



		static uint32 visit_stamp = 0;
		//visited.assign(mNodes.size(), 0);

		visit_stamp++;
		for (uint32 i = 0; i < dirty_node_count; ++i)
		{
			NodeID leaf_id = dirty_leave_nodes[i];
			if (visited[leaf_id] == visit_stamp)continue;
			NodeID curr_id = leaf_id;
			while (curr_id != kInvalidNode && visited[curr_id] != visit_stamp)
			{
				VX_ASSERT(curr_id < mNodes.size(), "corruptyed corrupted");
				VX_ASSERT(curr_id >= 0, "corruptyed corrupted");
				visited[curr_id] = visit_stamp;
				RefitNode(curr_id, mLeafNodeMargin);
				curr_id = mNodes[curr_id].parent;

				//hack
				//if (curr_id > 16383) break;
			}
			//RefitNode(leaf_id, mLeafNodeMargin);
		}

	


		if(mUpdateTreeCondition == UpdateNode::UpdateOnMarginChanged)
			ChangeUpdateState(mPrevUpdateTreeCondition);
	}

	template<typename BoundType>
	typename BVHTree<BoundType>::NodeID BVHTree<BoundType>::EmplaceNode(NodeID parent, const BoundType& bounds, Body* body)
	{
		VX_ASSERT_WARN(mNodes.size() != mNodes.capacity(), "About to resize, which would cause pointer to miss align. Rebuild!!!!");

		NodeID id = static_cast<NodeID>(mNodes.size());

		Node* node = &mNodes.emplace_back(parent, bounds, body);

		//quick id for hashing
		node->id = id;/*mNodes.size() - 1*/;
		RefitNode(id, mLeafNodeMargin);

#if defined(VX_PROFILE_BROAD)
		mStats.flatNodeSize = static_cast<uint>(mNodes.size());
		mStats.flatNodeCapacity = static_cast<uint>(mNodes.capacity());
#endif // defined(VX_PROFILE_BROAD)
		return id;
	}






	template<typename BoundType>
	void BVHTree<BoundType>::ComputeCollidingPairs(std::vector<BroadphasePair>& potential_pair)
	{
		VX_PROFILE_FUNCTION();
		if (mRootID == kInvalidNode) return;

#if defined(VX_PROFILE_BROAD)
		mStats.dynamic_dynamic_pair = 0;
		mStats.static_dynamic_pair = 0;
		mStats.dynamic_static_pair = 0;
		mStats.static_static_pair = 0;
		mStats.crossed_nodes = 0;
#endif // defined(VX_PROFILE_BROAD)

		struct Pair {
			NodeID a, b;
		};
		static constexpr int k_max_node_stack = 512;
		static Pair node_stack[k_max_node_stack];
#define NODE_STACK_GUARD(x) if (depth > k_max_node_stack - x) continue
		node_stack[0] = { mRootID, mRootID };
		int depth = 1;
		int stack_count = 0;

		int count = 0;
		int max_stack_size = 0;
		do
		{
			max_stack_size = VxMax(max_stack_size, depth);
			Pair p = node_stack[--depth];
			NodeID a_id = p.a;
			NodeID b_id = p.b;


			if (a_id == kInvalidNode || b_id == kInvalidNode)
				continue;


			Node& node_a = mNodes[a_id];
			Node& node_b = mNodes[b_id];

			if (!node_a.Overlaps(node_b)) continue; //prune

			if (a_id == b_id)
			{
				if (node_a.IsInternal())
				{
					NODE_STACK_GUARD(3);
					node_stack[depth++] = { node_a.children[0], node_a.children[1] };
					node_stack[depth++] = { node_a.children[0], node_a.children[0] };//h
					node_stack[depth++] = { node_a.children[1], node_a.children[1] };//h
				}
				continue;
			}


			const bool a_is_leaf = node_a.IsLeaf();
			const bool b_is_leaf = node_b.IsLeaf();

			if (a_is_leaf && b_is_leaf)
			{
				//since pointer just work with pointerr
				Body* b0 = node_a.body;
				Body* b1 = node_b.body;

				//if (!b0->IsDynamic() && b1->IsDynamic())
				//	std::swap(b1, b0);

				//Body::CanBodiesCollide(b0, b1);
				if (Body::CanBodiesCollide(*b0, *b1))
				{
					if(mBodyManager)
					{
						mBodyManager->GetBodySimStats(*b0).phase |= EBodySimphaseFlags::InBroadphase;
						mBodyManager->GetBodySimStats(*b1).phase |= EBodySimphaseFlags::InBroadphase;
					}

					potential_pair.emplace_back(b0, b1);
				}

				count++;
				continue;
			}


			if (!a_is_leaf && !b_is_leaf)
			{
				NODE_STACK_GUARD(4);
				/// 2 braches, no leave, cross both
				node_stack[depth++] = { node_a.children[0], node_b.children[0] };
				node_stack[depth++] = { node_a.children[0], node_b.children[1] };
				node_stack[depth++] = { node_a.children[1], node_b.children[0] };
				node_stack[depth++] = { node_a.children[1], node_b.children[1] };
				continue;
			}

			NODE_STACK_GUARD(2);

			if (a_is_leaf)
			{
				node_stack[depth++] = { a_id, node_b.children[0] };
				node_stack[depth++] = { a_id, node_b.children[1] };
			}
			else
			{
				node_stack[depth++] = { node_a.children[0],  b_id };
				node_stack[depth++] = { node_a.children[1],  b_id };
			}
		} while (depth);

#if defined(VX_PROFILE_BROAD)
		mStats.pairCount = count;
		mStats.actualPairCount = potential_pair.size();
		mStats.treeIterativePairStack = stack_count;
		mStats.frameMaxStackSize = max_stack_size;
		mStats.maxAttainedStackSize = VxMax(mStats.maxAttainedStackSize, static_cast<uint>(max_stack_size));
#endif // defined(VX_PROFILE_BROAD)
	}



	template<typename BoundType>
	void BVHTree<BoundType>::CastRay(const RayCast& ray_cast, WorldRayCastQuery& world_ray_ctx) const
	{
		static constexpr int k_max_node_stack = 512;
		static NodeID node_stack[k_max_node_stack];
#define NODE_STACK_GUARDR(x) if (top > k_max_node_stack - x) continue
		node_stack[0] = mRootID;

		int top = 1;
		int stack_count = 0;

		int count = 0;
		int max_stack_size = 0;
		do
		{
			NodeID curr_node_id = node_stack[--top];
			Node curr_node = mNodes[curr_node_id];

			if (curr_node.IsLeaf())
			{
				world_ray_ctx.VisitBody(*curr_node.body);

				if (world_ray_ctx.Complete())
					break;
			}
			else if (curr_node.IsInternal())
			{
				NODE_STACK_GUARDR(2);

				constexpr bool k_use_old_aabb_test = false;

				if constexpr(k_use_old_aabb_test)
				{
					Node child_node = mNodes[curr_node.children[0]];
					if (world_ray_ctx.VisitNode(child_node.bounds.mMin, child_node.bounds.mMax))
						node_stack[top++] = curr_node.children[0];

					child_node = mNodes[curr_node.children[1]];
					if (world_ray_ctx.VisitNode(child_node.bounds.mMin, child_node.bounds.mMax))
						node_stack[top++] = curr_node.children[1];
				}
				else
				{
					Node nA = mNodes[curr_node.children[0]];
					Vec3 _minA = nA.bounds.mMin;
					Vec3 _maxA = nA.bounds.mMax;

					Node nB = mNodes[curr_node.children[1]];
					Vec3 _minB = nB.bounds.mMin;
					Vec3 _maxB = nB.bounds.mMax;


					//load data
					Vec4 boundsX = Vec4(_minA.X(), _minB.X(), _maxA.X(), _maxB.X());
					Vec4 boundsY = Vec4(_minA.Y(), _minB.Y(), _maxA.Y(), _maxB.Y());
					Vec4 boundsZ = Vec4(_minA.Z(), _minB.Z(), _maxA.Z(), _maxB.Z());

					Float3 result = world_ray_ctx.VisitNode2(boundsX, boundsY, boundsZ);

					float tA = result.x;
					float tB = result.y;

					if (tA == kMaxf && tB == kMaxf)
						continue;

					//sort by distance
					if (tA != kMaxf && tB != kMaxf)
					{
						if(tA < tB)
						{
							node_stack[top++] = curr_node.children[1];
							node_stack[top++] = curr_node.children[0];
						}
						else
						{
							node_stack[top++] = curr_node.children[0];
							node_stack[top++] = curr_node.children[1];
						}
					}
					else if(tA != kMaxf)
						node_stack[top++] = curr_node.children[0];
					else 
						node_stack[top++] = curr_node.children[1];
				}

				//Node nA = mNodes[curr_node.children[0]];
				//Vec3 _minA = nA.bounds.mMin;
				//Vec3 _maxA = nA.bounds.mMax;

				//Node nB = mNodes[curr_node.children[1]];
				//Vec3 _minB = nB.bounds.mMin;
				//Vec3 _maxB = nB.bounds.mMax;


				////load data
				//Vec4 boundsX = Vec4(_minA.X(), _minB.X(), _maxA.X(), _maxB.X());
				//Vec4 boundsY = Vec4(_minA.Y(), _minB.Y(), _maxA.Y(), _maxB.Y());
				//Vec4 boundsZ = Vec4(_minA.Z(), _minB.Z(), _maxA.Z(), _maxB.Z());

				//Float3 result = world_ray_ctx.VisitNode2(boundsX, boundsY, boundsZ);

				//float tA = result.x;
				//float tB = result.y;

				//if (tA == kMaxf && tB == kMaxf)
				//	continue;

				////sort by distance
				//if (tA != kMaxf && tB != kMaxf)
				//{
				//	if(tA < tB)
				//	{
				//		node_stack[top++] = curr_node.children[1];
				//		node_stack[top++] = curr_node.children[0];
				//	}
				//	else
				//	{
				//		node_stack[top++] = curr_node.children[0];
				//		node_stack[top++] = curr_node.children[1];
				//	}
				//}
				//else if(tA != kMaxf)
				//	node_stack[top++] = curr_node.children[0];
				//else 
				//	node_stack[top++] = curr_node.children[1];
			}


		} while (top > 0);
	}


	template<typename BoundType>
	void BVHTree<BoundType>::RebuildBruteforceInsertion()
	{
		VX_PROFILE_FUNCTION();
		struct BuildProxy 
		{
			BoundType bounds;
			Body* body;
		};

		std::vector<BuildProxy> build_proxies;
		build_proxies.reserve(mStats.nodeCount);
		//collect leave
		for (const auto& n : mNodes)
		{
			if(n.IsLeaf())
				build_proxies.push_back({ n.bounds, n.body });
		}


		if (build_proxies.empty())
			return;

		//clear old node
		ClearNodes();
		for (auto& p : build_proxies)
			AddBody(p.body, p.bounds);
	}

	template<typename BoundType>
	void BVHTree<BoundType>::RebuildAllSAH(uint32 max_nodes)
	{
		VX_PROFILE_FUNCTION();
		struct BuildProxy
		{
			BoundType bounds; //32 bytes
			Vec3 centroid;//16 bytes
			Body* body;//8bytes
		};

		std::vector<BuildProxy> build_proxies;
		build_proxies.reserve(mStats.nodeCount);
		//collect leave
		for (int i = 0; i < mLeafNodeCount; ++i)
		{
			const auto& n = mNodes[mLeafNodeIDs[i]];
			build_proxies.push_back({ n.bounds, n.bounds.GetCenter(), n.body });
		}
		//for (const auto& n : mNodes)
		//{
		//	if (n.IsLeaf())
		//		build_proxies.push_back({ n.bounds, n.bounds.GetCenter(), n.body});
		//}

		if (build_proxies.empty())
			return;


		VX_ASSERT(uint32(build_proxies.size()) < max_nodes, "Rebuild Topdown SAH proxy requires size greater than max nodes");
		std::vector<Node> new_nodes;
		//new_nodes.reserve(build_proxies.size() * 2); //bad for insert when tree is swap due new node triggers memory allocation
		new_nodes.reserve(max_nodes);

		//const int kBucketCount = 12;
		struct BucketInfo
		{
			BoundType bounds;
			int count = 0;
		};

		auto bucket_index = [&](const Vec3& min_c, const Vec3& centroid, int axis,
			const float inv_extent) -> int {
				float t = (centroid[axis] - min_c[axis]) * inv_extent;
				int b = static_cast<int>(t * kBucketCount);

				//b = b < 0 ? 0 : b;
				//b = b >= kBucketCount ? kBucketCount - 1 : b;
				
				if (b < 0)b = 0;
				else if (b >= kBucketCount) b = kBucketCount - 1;
				return b;
				//b = Clamp(b, 0, kBucketCount - 1);
			};
		

		/// require a new leaf nodes
		/// quick hack 
		NodeID* new_leaf_nodesID = new NodeID[mBodyManager->MaxBodies()];
		uint32 new_leaf_node_count = 0;

		auto Track_Leaf_Node = [&](NodeID node_id) -> bool
			{
				Node& leaf_node = new_nodes[node_id];
				if (leaf_node.IsInternal())
					return false;
				leaf_node.leafNodeIdx = new_leaf_node_count;
				new_leaf_nodesID[new_leaf_node_count++] = leaf_node.id;
				return true;
			};


		auto Remove_Tracking_Node = [&](NodeID node_id) -> bool
			{
				Node& node = new_nodes[node_id];
				if (node.leafNodeIdx <= 0) return false;

				//move node to the end 
				int idx = node.leafNodeIdx;
				int last_node_idx = new_leaf_node_count - 1;

				NodeID last_leaf = new_leaf_nodesID[last_node_idx];

				//swap
				new_leaf_nodesID[idx] = last_leaf;
				new_nodes[last_leaf].leafNodeIdx = idx;

				new_leaf_node_count--;

				return true;
			};

		auto make_internal = [&](NodeID left, NodeID right, uint32 depth) -> NodeID
			{
				Node internal;
				internal.body = nullptr;
				internal.children[0] = left;
				internal.children[1] = right;
				internal.parent = kInvalidNode;
				internal.bounds = new_nodes[left].bounds.Merged(new_nodes[right].bounds);
				internal.id = static_cast<NodeID>(new_nodes.size());

				//internal.height = 1 + std::max(new_nodes[left].depth,
				//							  new_nodes[right].depth);
				internal.depth = depth;

				//Remove_Tracking_Node(internal.id); its a new node so ???


				new_nodes.push_back(internal);
				NodeID id = static_cast<NodeID>(new_nodes.size() - 1);
				new_nodes[left].parent = id;
				new_nodes[right].parent = id;


				return id;
			};

		BucketInfo buckets[kBucketCount];
		auto build_range = [&](auto&& self, size_t start, size_t end, uint32 depth) -> NodeID
			{
				size_t count = end - start;
				if (count == 1)
				{
					Node leaf;
					leaf.children[0] = leaf.children[1] = leaf.parent = kInvalidNode;
					leaf.body = build_proxies[start].body;
					leaf.bounds = build_proxies[start].bounds;
					leaf.id = static_cast<NodeID>(new_nodes.size());
					leaf.depth = depth;
					new_nodes.push_back(leaf);
					//return static_cast<NodeID>(new_nodes.size() - 1);

					Track_Leaf_Node(leaf.id);
					return leaf.id;
				}

				//compute centroid bounds
				BoundType centroid_bounds = build_proxies[start].bounds;
				Vec3 min_c = build_proxies[start].centroid;
				Vec3 max_c = build_proxies[start].centroid;
				for (size_t i = start + 1; i < end; ++i)
				{
					min_c = Vec3::Min(min_c, build_proxies[i].centroid);
					max_c = Vec3::Max(max_c, build_proxies[i].centroid);
				}
				Vec3 extent = max_c - min_c;
				int axis = static_cast<int>(extent.MaxAxis());
				if (extent[axis] <= 1e-6f)
				{
					//degenerate make leaf with all proxies
					size_t mid = start + count / 2;
					std::nth_element(build_proxies.begin() + start, build_proxies.begin() + mid,
						build_proxies.begin() + end, [axis](const BuildProxy& a, const BuildProxy& b) {
						return a.centroid[axis] < b.centroid[axis];
					});


					NodeID left = self(self, start, mid, depth + 1);
					NodeID right = self(self, mid, end, depth + 1);
					return make_internal(left, right, depth);
				}


				//bucketisation
				memset(buckets, 0, sizeof(buckets));
				for (int b = 0; b < kBucketCount; ++b)
					buckets[b].count = 0;
				//compute bucket index for eack proxy
				//Assifmn proxies to buckets 
				//this loop is often The mosr expensive part of BVH building 
				//we compute bucket idx -> merge the bounds -> increment count
				const float inv_extent = 1.0f / (extent[axis]);
				for (size_t i = start; i < end; ++i)
				{
					int b = bucket_index(min_c, build_proxies[i].centroid, axis, inv_extent);
					if (buckets[b].count == 0)
						buckets[b].bounds = build_proxies[i].bounds;
					else
						buckets[b].bounds = buckets[b].bounds.Merged(build_proxies[i].bounds);

					buckets[b].count++;
				}

				////////
				// Prefiz / suffix scans ro compute SAH in 0(B)
				////////
				//left side aggregates [0....i]
				BoundType left_bounds[kBucketCount];
				int left_count[kBucketCount];
				//left side aggregates [i.... buncket count -1]
				BoundType right_bounds[kBucketCount];
				int right_count[kBucketCount];
				///////////////////////////
				// Build left 
				///////////////////////////
				bool left_init = false;
				int cnt = 0;
				for (int i = 0; i < kBucketCount; ++i)
				{
					if(buckets[i].count > 0)
					{
						if (!left_init)
						{
							left_bounds[i] = buckets[i].bounds;
							left_init = true;
						}
						else
							left_bounds[i] = left_bounds[i - 1].Merged(buckets[i].bounds);
						cnt += buckets[i].count;
					}
					else
					{
						if (i > 0 && left_init)
							left_bounds[i] = left_bounds[i - 1];
					}
					left_count[i] = cnt;

				}

				///////////////////////////
				// Build right
				///////////////////////////
				bool right_init = false;
				cnt = 0;
				for (int i = kBucketCount - 1; i>=0; --i)
				{
					if(buckets[i].count > 0)
					{
						if (!right_init)
						{
							right_bounds[i] = buckets[i].bounds;
							right_init = true;
						}
						else
							right_bounds[i] = right_bounds[i + 1].Merged(buckets[i].bounds);
						cnt += buckets[i].count;
					}
					else if ((i < kBucketCount - 1) && right_init)
					{
						if (i < kBucketCount - 1 && right_init)
							right_bounds[i] = right_bounds[i + 1];
					}

					right_count[i] = cnt;

				}

				
				//evaluate split cost
				float best_cost = std::numeric_limits<float>::infinity();
				int best_split = -1;
				for (int s = 1; s < kBucketCount; ++s)
				{
					const int lc = left_count[s - 1];
					const int rc = right_count[s];

					if (lc == 0 || rc == 0)
						continue;

					const float cost =
						lc * left_bounds[s - 1].GetSurfaceArea() +
						rc * right_bounds[s].GetSurfaceArea();

					if (cost < best_cost)
					{
						best_cost = cost;
						best_split = s;
					}
				}

				//if no valid splir (all proxies in on ebucket, fallback to mdeian split nby centroiud 
				if (best_split == -1)
				{
					size_t mid = start + count / 2;
					std::nth_element(build_proxies.begin() + start, build_proxies.begin() + mid,
						build_proxies.begin() + end, [axis](const BuildProxy& a, const BuildProxy& b) {
						//build_proxies.begin() + end, [&](const BuildProxy& a, const BuildProxy& b)) {
						return a.centroid[axis] < b.centroid[axis];
					});
					NodeID left = self(self, start, mid, depth + 1);
					NodeID right = self(self, mid, end, depth + 1);
					return make_internal(left, right, depth);
				}


				//////////////
				// Partition proxies by best split
				//////////////
				auto it_mid = std::partition(build_proxies.begin() + start, build_proxies.begin() + end,
					[&](const BuildProxy& p)
				{
					int b = bucket_index(min_c, p.centroid, axis, inv_extent);
					return b < best_split;
				});

				size_t mid = static_cast<size_t>(std::distance(build_proxies.begin(), it_mid));
				//if partition degenerated, fall to median by centroid
				if (mid == start || mid == end)
				{
					mid = start + count / 2;
					std::nth_element(build_proxies.begin() + start, build_proxies.begin() + mid,
						build_proxies.begin() + end, 
						[axis](const BuildProxy& a, const BuildProxy& b) 
					{
						return a.centroid[axis] < b.centroid[axis];
					});
				}

				NodeID left_id = self(self, start, mid, depth + 1);
				NodeID right_id = self(self, mid, end, depth + 1);
				return make_internal(left_id, right_id, depth);
				
			};//end build range

			//build whole tree
			NodeID root = build_range(build_range, 0, build_proxies.size(), 0);
			//replace old with nodes with newNode atomically (or swap)
			mNodes.swap(new_nodes);
			//recompute root id if needed, update stats
			mRootID = root;

			//swap leaf nodes 
			delete[] mLeafNodeIDs;
			mLeafNodeIDs = new_leaf_nodesID;
			mLeafNodeCount = new_leaf_node_count;

#if defined(VX_PROFILE_BROAD)
			mStats.nodeCount = static_cast<int>(mNodes.size());
			mStats.leafCount = static_cast<int>(build_proxies.size());
#endif // defined(VX_PROFILE_BROAD)
	}


	template<typename BoundType>
	void BVHTree<BoundType>::DebugTree(NodeID id, int depth)
	{
		if (id == kInvalidNode) return;

		Node& node = mNodes[id];

		std::string pad(depth * 2, ' ');
		VX_LOG_DEBUG(pad, (node.IsLeaf() ? "Leaf" : "Internal"),
			" | body: ", (node.body ? "yes" : "no"),
			" | children: ", node.children[0],
			",", node.children[1], ".");// ", id: ", node->id);

		//std::string pad(depth * 2, ' ');
		//VX_DEBUG(pad, (node->IsLeaf() ? "Leaf" : "Internal"),
		//	" | body: ", (node->body ? "yes" : "no"),
		//	" | children: ", (node->children[0] != nullptr),
		//	",", (node->children[1] != nullptr), ".");// ", id: ", node->id);

		DebugTree(node.children[0], depth + 1);
		DebugTree(node.children[1], depth + 1);
	}



#if defined(VX_PROFILE_BROAD)
	template<typename BoundType>
	StackString<512> BVHTree<BoundType>::BVHStats::AsString() const
	{

		StackString<512> text("node count: ");

		text << nodeCount;
		text << "\nleaf count: ";
		text << leafCount;
		text << "\n\npair count: ";
		text << pairCount;

		text << "\ndynamic_dynamic_pair count: ";
		text << dynamic_dynamic_pair;
		text <<  "\nstatic_dynamic_pair count: ";
		text << static_dynamic_pair;
		text <<  "\ndynamic_static_pair count: ";
		text << dynamic_static_pair;
		text <<  "\nstatic_static_pair count: ";
		text << static_static_pair;

		text <<  "\n\nDirty nodes: ";
		text << dirty_nodes;
		text <<  "\n\nDirty Leaf nodes: ";
		text << dirty_leaf_nodes;
		text <<  "\n\nMax dirty nodes: ";
		text << max_dirty_nodes;
		text <<  "\n\nCrossed nodes: ";
		text << crossed_nodes;


		text <<  "\nmax attainable pair count: ";
		text << MaxAttainablePairCount();
		text <<  "\nactual pair count: ";
		text << actualPairCount;
		text <<  "\n\nTree Transversal Node Pair Stack count: ";
		text << treeIterativePairStack;
		text <<  "\n\nflat node capacity: ";
		text << flatNodeCapacity;
		text <<  "\nflat node size: ";
		text << flatNodeSize;
		text <<  "\nframe max stack size: ";
		text << frameMaxStackSize;
		text <<  "\nmax attained stack size: ";
		text << maxAttainedStackSize;

		return text;
	}
#endif // defined(VX_PROFILE_BROAD)
} ///namespace vx


namespace vx {
	template class BVHTree<AABB>;
	template class BVHTree<AABB>::Node;
}





//template<typename BoundType>
//void BVHTree<BoundType>::Node::Insert(const BVHContext& bvh_context, Body* new_body, const BoundType& new_bounds)
//{
//	//if (IsLeafWithNoData())
//	//{
//	//	body = new_body;
//	//	bounds = new_bounds;
//	//	bvh_context.stats->leafCount += 1; 
//	//	return;
//	//}

//	/// Surface Area Heuristic (SAH)
//	auto compute_surface_area_growth = [](const BoundType& a, const BoundType& b) {
//		return a.Merged(b).GetSurfaceArea() - a.GetSurfaceArea();
//		};

//	/// If we are a leaf, then teh only optin is to spawn 
//	/// two new children and place the new body in one
//	if (IsLeaf())
//	{
//		bvh_context.stats->nodeCount += 2;
//		bvh_context.stats->leafCount += 1;
//		/// chilfd id a copy of us
//		children[0] = bvh_context.tree->EmplaceNode(this, bounds, body); //<-- copy current data over
//		/// second child holds the new data(node/body)
//		children[1] = bvh_context.tree->EmplaceNode(this, new_bounds, new_body);

//		//its better to compute a better left or right
//		//Node* node0;
//		//Node* node1;
//		float left_growth_cost = compute_surface_area_growth(children[0]->bounds, children[1]->bounds);
//		float right_growth_cost = compute_surface_area_growth(children[1]->bounds, children[0]->bounds);

//		if (right_growth_cost < left_growth_cost)
//			std::swap(children[0], children[1]);
//		/// Now this lose current data
//		/// excepciaally body point and refit bounds
//		body = nullptr;
//		RecomputeBounds();
//	}
//	else
//	{
//		/// otherwise, work out which child gets to keep/transvasre 
//		/// towards the insterd body
//		/// target node that requires less growth
//		float left_growth = compute_surface_area_growth(children[0]->bounds, new_bounds);
//		float right_growth = compute_surface_area_growth(children[1]->bounds, new_bounds);

//		Node* target_node = (left_growth <= right_growth) ? children[0] : children[1];
//		//children[0]->Insert(bvh_context, new_body, new_bounds);
//		target_node->Insert(bvh_context, new_body, new_bounds);
//		RecomputeBounds();
//	}
//}
//template<typename BoundType>
//uint BVHTree<BoundType>::Node::GetPotentialContactWith(const Node* rhs, std::vector<BroadphasePair>& potential_pair)
//{
//	///Early out if we dont overlap 
//	if (!Overlaps(rhs))
//		return 0;

//	///If we're both at leaf node, then we have a potentail contact
//	/// because parent node should hold leaves with two leaves with children 
//	if (IsLeaf() && rhs->IsLeaf())
//	{
//		BroadphasePair new_pair(body, rhs->body);
//		potential_pair.push_back(new_pair);
//		return 1;
//	}

//	uint count = 0;
//	if (!IsLeaf() && (rhs->IsLeaf() || bounds.GetVolume() >= rhs->bounds.GetVolume()))
//	{
//		///recursive solve
//		count += children[0]->GetPotentialContactWith(rhs, potential_pair);

//		count += children[1]->GetPotentialContactWith(rhs, potential_pair);

//		return count;
//	}
//	else
//	{
//		///recursive solve
//		count += GetPotentialContactWith(rhs->children[0], potential_pair);

//		count += GetPotentialContactWith(rhs->children[1], potential_pair);

//	}
//	return count;
//}
//template<typename BoundType>
//uint BVHTree<BoundType>::Node::GetPotentialContact(const BVHContext& bvh_context, std::vector<BroadphasePair>& potential_pair)
//{
//	///Early out if at leaf
//	if (IsLeaf())
//		return 0;

//	///get potential contact from one of our children
//	uint contact_count = children[0]->GetPotentialContactWith(children[1], potential_pair);

//	contact_count += children[0]->GetPotentialContact(bvh_context, potential_pair);
//	contact_count += children[1]->GetPotentialContact(bvh_context, potential_pair);
//	bvh_context.stats->pairCount = contact_count;
//	bvh_context.stats->actualPairCount = potential_pair.size();


//	//expected count/max pair

//	return contact_count;
//}










//template<typename BoundType>
//uint BVHTree<BoundType>::ComputePairs(const BVHContext& bvh_context, Node* root_a, Node* root_b, std::vector<BroadphasePair>& potential_pair)
//{
//	if (!root_a || !root_b)
//		return 0;


//	bvh_context.stats->dynamic_dynamic_pair = 0;
//	bvh_context.stats->static_dynamic_pair = 0;
//	bvh_context.stats->dynamic_static_pair = 0;
//	bvh_context.stats->static_static_pair = 0;

//	// Pair traversal 
//	struct NodePair { Node* a; Node* b; };
//	std::vector<NodePair> node_stack;
//	//constexpr uint kStackSize = 128;
//	//std::stack<NodePair, kStackSize> node_stack;
//	node_stack.reserve(bvh_context.stats->nodeCount * 2); //<-- later this can be limit

//	uint stacked_node_pair_count = 0;
//	auto push_pair = [&](Node* a, Node* b) {
//		if (!a || !b)return;
//		node_stack.push_back({ a, b });
//		stacked_node_pair_count++;
//		};

//	push_pair(root_a, root_b);

//	auto push_node = [&](Node* n) {
//		if (!n) return;
//		push_pair(n->children[0], n->children[1]);
//		};

//	//expaned nodes in stack
//	std::vector<uint8> crossed_nodes(bvh_context.stats->nodeCount, 0);

//	auto cross_children_node = [&](Node* n) {
//		//node has to vaild & not the leaf node
//		if (!n || n->IsLeaf()) return;
//		//check already crossed node
//		uint32 id = n->id;
//		if (crossed_nodes[id]) return;
//		//push node to stack for pair
//		push_node(n);
//		crossed_nodes[id] = 1;
//		};

//	int count = 0;
//	do
//	{
//		auto [a, b] = node_stack.back();
//		node_stack.pop_back();

//		if (!a || !b || a == b)
//			continue;


//		/// proxies check 
//		/// if no overlap, early out
//		/// and cross childrens
//		if (!a->Overlaps(b))
//		{
//			///Even with no overlap 
//			/// pupolate the stack with our children 
//			/// for potental collsion
//			cross_children_node(a);
//			cross_children_node(b);
//			continue;
//		}

//		/// if both leaf 
//		/// allocate / create pair
//		if (a->IsLeaf() && b->IsLeaf())
//		{
//			const Body& b0 = *a->body;
//			const Body& b1 = *b->body;


//			if (!Body::CanBodiesCollide(b0, b1))
//				continue;

//			auto& bvh_stat = bvh_context.stats;
//			bvh_stat->dynamic_dynamic_pair += uint(b0.IsDynamic() && b1.IsDynamic());
//			bvh_stat->static_dynamic_pair += uint(!b0.IsDynamic() && b1.IsDynamic());
//			bvh_stat->dynamic_static_pair += uint(b0.IsDynamic() && !b1.IsDynamic());
//			bvh_stat->static_static_pair += uint(!b0.IsDynamic() && !b1.IsDynamic());
//			//VX_ASSERT_WARN(a->body->IsDynamic(), "body a is static");
//			//guarentee pair constiency 
//			//if (a->id > b->id)
//				//std::swap(a, b);
//			potential_pair.emplace_back(a->body, b->body);
//			count++;
//			continue;
//		}


//		/// 1 branch, 1 leaf, cross branch (a & b, left & right)
//		if (!a->IsLeaf() && b->IsLeaf())
//		{
//			push_pair(a->children[0], b);
//			push_pair(a->children[1], b);

//			/// cross branch a if its children is
//			/// not already crossed
//			cross_children_node(a);
//		}
//		else if (a->IsLeaf() && !b->IsLeaf())
//		{
//			push_pair(a, b->children[0]);
//			push_pair(a, b->children[1]);

//			/// cross branch b if its children is
//			/// not already crossed
//			cross_children_node(b);
//		}
//		else
//		{

//			/// 2 braches, no leave, cross both
//			push_pair(a->children[0], b->children[0]);
//			push_pair(a->children[0], b->children[1]);
//			push_pair(a->children[1], b->children[0]);
//			push_pair(a->children[1], b->children[1]);


//			/// cross branch a & b 
//			/// if its children is
//			/// not already crossed
//			cross_children_node(a);
//			cross_children_node(b);
//		}
//	} while (!node_stack.empty());

//	bvh_context.stats->pairCount = count;
//	bvh_context.stats->actualPairCount = potential_pair.size();
//	bvh_context.stats->treeIterativePairStack = stacked_node_pair_count;
//	return count;
//}