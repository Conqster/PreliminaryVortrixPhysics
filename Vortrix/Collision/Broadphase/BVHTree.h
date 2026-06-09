#pragma once
#include <Vortrix.h>

#include "Broadphase.h"

namespace vx
{
	class Body;
	class BodyManager;
	struct BroadphasePair;


	enum class UpdateNode : uint8
	{
		Normal,
		UpdateIfMoved,
		AlwaysUpdate,
		UpdateOnMarginChanged
	};


	template<typename BoundType>
	class BVHTree
	{
	public:

		void Init(BodyManager* in_body_manager, uint node_limit = 512);

		struct BVHContext
		{
			BVHTree* tree;
			struct BVHStats* stats;
		};




		static constexpr int kBucketCount = 12;
		static constexpr int kMaxDirtyNodeUpdate = 128;
		//using NodeID = typename Node::NodeID;
		using NodeID = uint32;
		static constexpr NodeID kInvalidNode = UINT32_MAX;
		/// struct represnring nodes in tree
		/// node should only be a datat continer
		struct Node
		{

			Node() = default;
			Node(NodeID _parent, BoundType _bounds, Body* _body) :
				children{ kInvalidNode, kInvalidNode },
				parent(_parent),
				bounds(_bounds),
				body(_body) {
			}

			///alignment for cache line
			/// bounds to 32 bytes
			/// body pointer 8 bytes {remining 24 bytes for align}
			/// children 4 bytes x 2 = 8 bytes {remining 16 bytes for align}
			/// parent 4 bytes {remining 12 bytes for align}
			/// id 4 bytes {remining 8 bytes for align}
			/// depth 4 bytes {remining 4 bytes for align}
			/// dirty 1 byte {remining 3 bytes for align}
			/// add 3 bytes padding to make 64 bytes 
			/// padding boolean[3] to make up to 64 bytes


			/// hold a single bounds encomassinf 
			/// all descendants of this node
			BoundType bounds;									//32 bytes	 might remove later then use id to access
			/// -1 if node is not leave node 
			/// >= 0 for leaves 
			//uint objectID = -1;
			Body* body = nullptr;								//32 + 8 = 40 bytes change to body idx

			NodeID children[2] = { kInvalidNode, kInvalidNode };//40 + 8 = 48 bytes 
			NodeID parent = kInvalidNode;						//48 + 4 = 52 bytes 


			NodeID id = 0;										//52 + 4 = 56 bytes

			//quick check 
			int leafNodeIdx = -1;								//56 + 4 = 60 bytes 

			bool IsInvalid() const { return id == kInvalidNode; }

			///use to remove node without having to clear Nodebuffer
			void Invalidate()
			{
				//id = kInvalidNode;
				//body = nullptr;
				//leafNodeIdx = -1;
				//children[0] = kInvalidNode;
				//children[1] = kInvalidNode;
				//parent = kInvalidNode;
				//depth = 
				//bounds.Reset();
				*this = Node();
				id = kInvalidNode;
			}

			//node becomes dirty when leaf moves
			//bool dirty = false;
			//bool pad[3];
#if defined(VX_PROFILE_BROAD)
			uint32 depth = 0;
#endif //VX_PROFILE_BROAD 
			bool IsLeaf() const { return body != nullptr; }
			bool IsInternal() const { return !IsLeaf(); }
			bool Overlaps(const Node& rhs) const { return bounds.Overlaps(rhs.bounds); }
			/// mark dirty and propagete up (to root)
			/// if mark dirty it means parent is already 
			/// dirty early out
			//void MarkDirty();
			/// only refit 
			/// leaf node, but recompute bounds of its parent
			//void Refit(float body_bounds_margin = 0.02f); //20cm;
			//void RecomputeBounds();
			~Node() {}
		};

		static_assert(sizeof(Node) <= 64);
		using Nodebuffer = std::vector<Node>;
		Nodebuffer mNodes;

		size_t NodeCount() const { return mNodes.size(); }

		inline const Node* GetRoot() const { return &mNodes[mRootID]; }
		inline NodeID GetRootID() const { return mRootID; }


		inline const Node* GetNode(const NodeID& id) const
		{
			//VX_ASSERT_WARN(false, "Invalid id & out of bounds");
			VX_ASSERT_WARN((id < mNodes.size() && id != kInvalidNode), "Invalid id & out of bounds");
			return &mNodes[id];
		}

		inline Node& GetNode2(const NodeID& id)
		{
			VX_ASSERT_WARN((id < mNodes.size() && id != kInvalidNode), "Invalid id & out of bounds");
			return mNodes[id];
		}


		//void Insert(Body* body, const BoundType& bounds){AddBody(body, bounds);}


		void AddBody(Body* body, const BoundType& bounds);
		void RemoveBody(const BodyID& id);
		void DeleteNode(Node& node);
		void DeleteAndUpdateParent(const NodeID id);

		/// this need to be reset when a fresh tree is build
		/// to avoid collision 
		std::vector<uint32> mFreedIdxs;
		VX_INLINE void ResetFreedNodeIdxList() { mFreedIdxs.clear(); }
		
		/// new to change this to allocate 
		NodeID EmplaceNode(NodeID parent, const BoundType& bounds, Body* body);

		const Nodebuffer& GetNodes() const { return mNodes; }


		void DebugTree(NodeID id, int depth = 0);



		///Utilites tracking tree
	public:
#if defined(VX_PROFILE_BROAD)
		struct BVHStats
		{
			uint nodeCount = 0;
			uint leafCount = 0;
			uint pairCount = 0;
			uint actualPairCount = 0;

			uint treeIterativePairStack = 0;

			uint flatNodeCapacity = 0;
			uint flatNodeSize = 0;

			uint dynamic_dynamic_pair = 0;
			uint static_dynamic_pair = 0;
			uint dynamic_static_pair = 0;
			uint static_static_pair = 0;

			uint dirty_nodes = 0;
			uint dirty_leaf_nodes = 0;
			uint max_dirty_nodes = 0;
			uint crossed_nodes = 0;

			uint frameMaxStackSize = 0;
			uint maxAttainedStackSize = 0;

			/// num = (N x (N - 1))/2
			/// where n is the number of leaf nodes
			uint MaxAttainablePairCount() const { return static_cast<uint>((leafCount * (leafCount - 1)) * 0.5f); }
			StackString<512> AsString() const;
		};
#endif //#if defined(VX_PROFILE_BROAD)


		float ComputeDepthImbalance()
		{
			size_t max_depth = 0;
			size_t sum_depth = 0;
			size_t leaf_count = 0;

			for (int i = 0; i < mLeafNodeCount; ++i)
			{
				size_t d = mNodes[mLeafNodeIDs[i]].depth;
				max_depth = VxMax(max_depth, d);
				sum_depth += d;
				leaf_count++;
			}
			float avg_depth = static_cast<float>(sum_depth) / leaf_count;
			float imbalance_ratio = max_depth / avg_depth - 1.0f;

			return imbalance_ratio;
		}
		float ComputeAverageJump()
		{
			//avg jump 
			//avg jump = (1/num of nodes) * 
			size_t total_jump = 0;
			size_t count = 0;


			static std::vector<NodeID> debug_stack;
			debug_stack.reserve(128);
			debug_stack.clear();
			Node* previous_node = nullptr;
			debug_stack.push_back(mRootID);
			while (!debug_stack.empty())
			{
				Node* current = &mNodes[debug_stack.back()];
				debug_stack.pop_back();

				if (previous_node)
				{
					size_t jump = VxAbs(reinterpret_cast<uintptr>(current) -
						reinterpret_cast<uintptr>(previous_node)) /
						sizeof(Node);
					total_jump += jump;
					count++;
				}

				previous_node = current;

				if (!current->IsLeaf())
				{
					debug_stack.push_back(current->children[1]);
					debug_stack.push_back(current->children[0]);
				}
			}
			return (static_cast<float>(total_jump) / count);
		}

		void ComputeCollidingPairs(PhysicsStepContext& physics_ctx, BroadphasePair* io_pairs, uint32& io_count);

		void UpdateDirtyNodes(const NodeID* dirty_leave_nodes, uint32 dirty_node_count/*uint32 last_broadphase_step*/);
		void RebuildBruteforceInsertion();
		void RebuildAllSAH(uint32 max_nodes);

		BVHContext GetContext() { return { this, &mStats }; }

		/// already allocate new node but use below to assign datas
		/// like parent, etc
		void InsertNode(const BVHContext& bvh_context, NodeID new_node, NodeID tranverse_node);

		/// only refit 
		/// leaf node, but recompute bounds of its parent
		void RefitNode(NodeID node_id, float bounds_margin);
		void RecomputeBounds(const NodeID& node_id);

		void SetLeafNodeMarginThreshold(float v) { mLeafNodeMargin = v; ChangeUpdateState(UpdateNode::UpdateOnMarginChanged); }


		void CastRay(const RayCast& ray_cast, WorldRayCastQuery& world_ray_ctx) const;


		void ClearNodes()
		{
			mNodes.clear();
			mRootID = kInvalidNode;

			mStats = {};
		}

		//BVHTree class
		//...
		~BVHTree()
		{
			mNodes.clear();
			delete[] mLeafNodeIDs;
		}


		UpdateNode GetNodeUpdateState() const { return mUpdateTreeCondition; }

		NodeID* GetLeafNodeIDs() const { return mLeafNodeIDs; }
		uint32 GetLeafNodeCount() const { return mLeafNodeCount; }
		BoundType mLeafNodesBound = {};
	private:
		NodeID mRootID = kInvalidNode;
		//std::vector<Node> mFlatNodes;
#if defined(VX_PROFILE_BROAD)
		BVHStats mStats;
#endif //#if defined(VX_PROFILE_BROAD)

		float mLeafNodeMargin = 0.2;
		
		UpdateNode mUpdateTreeCondition = UpdateNode::Normal;
		UpdateNode mPrevUpdateTreeCondition = UpdateNode::Normal;

		void ChangeUpdateState(UpdateNode state)
		{
			mPrevUpdateTreeCondition = mUpdateTreeCondition;
			mUpdateTreeCondition = state;
		}

		//std::vector<uint8> crossed_nodes;
		//// Pair traversal 
		//struct NodePair { NodeID a; NodeID b; };
		//std::vector<NodePair> mNodeStack;

		//used for bodies simulation stat update
		BodyManager* mBodyManager = nullptr;
		NodeID* mLeafNodeIDs = nullptr;
		uint32 mLeafNodeCount = 0;

		bool TrackLeafNode(NodeID node_id) //pass NodeID small memory ref
		{
			Node& leaf_node = mNodes[node_id];
			if (leaf_node.IsInternal())
				return false;
			leaf_node.leafNodeIdx = mLeafNodeCount;
			mLeafNodeIDs[mLeafNodeCount++] = leaf_node.id;

			return true;
		}


		bool RemoveTrackingNode(NodeID node_id) //pass NodeID small memory ref
		{
			VX_ASSERT_WARN_RETURN(node_id != kInvalidNode, false, "Invalid node");
			const Node& node = mNodes[node_id];
			if (node.leafNodeIdx <= -1) return false;

			//move node to the end 
			int idx = node.leafNodeIdx;
			int last_node_idx = mLeafNodeCount - 1;

			NodeID last_leaf = mLeafNodeIDs[last_node_idx];

			//swap
			mLeafNodeIDs[idx] = last_leaf;
			mNodes[last_leaf].leafNodeIdx = idx;
			mLeafNodeCount--;
			//reset node
			mNodes[node_id].leafNodeIdx = -1;


			return true;
		}
	};



	//extern template class BVHTree<AABB>;
	//extern template struct BVHTree<AABB>::Node;
}