#pragma once


#include "Vortrix.h"
#include "PhysicsWorldSettings.h"

#include "Collision/Broadphase/BroadphasePair.h"
#include "Collision/Broadphase/Broadphase.h"

#include "Dynamics/Body/BodyManager.h"

#include "Dynamics/ConstraintSolver/ContactConstraintSolver.h"
#include "Dynamics/Body/BodyID.h"

#include "Collision/RayCast.h"
#include "Collision/WorldQuery.h"

#include "Dynamics/ConstraintCoordinator.h"

#include "Collision/Narrowphase/NarrowphaseQuery.h"

////Things to do 
/// Implement 
///		MotionDynamics
/// fix shapes; especially box; function needs to be in local space
/// 
/// 

/// 
/// a deadly bug, you can just set the desnsity of a shap ein body manage 
/// the shape might be shared between multiple bodies

/// 
/// 
/// Later moving to Island building 
/// So do not make/setup constraint per success narrowphase
/// 
/// 
/// check bvh insight
/// 
/// 
/// FIX 
/// dynamic reallocation for BVH rebuild, 
/// ways centralise approapiate data to tree - broadphase system 
/// 
/// i.e 2 trees - both preallocated memotry.
/// then rebuild happen on tree2; no reallocation; just buiilf with 
/// avaliable resources.  
/// 
/// then swap main tree after construction. 
/// 
/// this would help to waste of memory
/// 
/// 
/// MULTIPLE SPHERE AT A TIME, CAUSE NAN IS SOLVER 
/// 
/// 
/// might need to fix this as 1k box then spawn capsule all 1k becomes capsile 
/// 					if(hack_shape == nullptr)
//{
//	body_settings.shape = mShapeArena.AllocateObject<BoxShape>(vx::Vec3::LoadFloat3Raw(mNewPhyObjectSettings.halfExtents));
//	hack_shape = body_settings.shape;
//					}
//					else
//						body_settings.shape = hack_shape;
//				}
//				else if (mNewPhyObjectSettings.bodyShape == vx::EShapeType::Capsule)
//				{
//					auto& he = mNewPhyObjectSettings.halfExtents;
//					body_settings.shape = mShapeArena.AllocateObject<CapsuleShape>(he.x, he.y);
//				}
//				else
//					//body_settings.shape = mShapeArena.AllocateObject<SphereShape>(mNewPhyObjectSettings.halfExtents.x);
//						body_settings.shape = new SphereShape(mNewPhyObjectSettings.halfExtents.x);


///
/// CONVERT MOUSE INTERACTION TO CONSTRAINT BASED
/// MEMOPRY ALLOC
// make body shape ref counted 

class DebugGizmosRenderer;
class Renderer;
namespace vx
{
	class Body;
	class BodySettings;
	class Broadphase;

	class AABB;
	template<typename T>
	class BVHBroadphase;

	class NarrowphaseQuery;

	enum class EShapeType : uint8;
	struct RenderSettings;
	//enum class ERenderInstanceFlags : uint32;

	class VX_API PhysicsWorld : NonCopyable
	{
	public:
		PhysicsWorld(const PhysicsWorldSettings& in_settings);
		~PhysicsWorld();

		static void CreateSimpleWorld(PhysicsWorld* io_world);

		static void GenerateWorldDefaultConfig(int& o_max_bodies, int& o_max_body_pairs, int& o_max_contact_constraint);
		void Init(float max_bodies, float max_body_pairs, float max_contact_constraint);

		Body* CreateBody(const BodySettings& body_setting);
		void RemoveBody(const BodyID& id);

		////////////////////////////////////////////////////////////////////////
		/// Step Simulation
		/// 
		/// a single step into physics frame 
		void StepSimulation(float dt);
		template<EShapeType Type>
		void OnDrawBody(const Body& body, Renderer* draw_renderer, 
			const RenderSettings& setting, const Colour& c = Colour::sMagenta);
		void OnDrawBodies(Renderer* draw_renderer, const RenderSettings& setting);
		void OnDebugDraw(DebugGizmosRenderer* debug_renderer);

		template<typename T>
		T* CreateConstraintT(const T& constraint) { return mConstraintCoordinator.CreateT(constraint); }
		template<typename T>
		void CreateConstraintsT(const T* constraint_Ts, uint32 count) { return mConstraintCoordinator.CreateT(constraint_Ts, count); }
		void AddConstraint(Constraint* constraint) { return mConstraintCoordinator.Add(&constraint, 1); }
		void RemoveConstraint(Constraint* constraint) { return mConstraintCoordinator.Remove(&constraint, 1); }

		Constraints& GetConstraints() { return mConstraintCoordinator.GetConstraints(); }

		void QuickDebugDrawInertia(DebugGizmosRenderer* debug_renderer);

		BodyVector& GetBodies() { return mBodyManager.GetBodies(); }

		const PhysicsWorldSettings& GetSettings() const { return mSettings; }
		PhysicsWorldSettings& GetSettings() { return mSettings; }

		const CollisionResolutionStat GetNarrowphaseStats() const;

		const ContactConstraintSolver::ContactConstraintSolverStat& GetContactConstraintSolverStats() const 
		{ return mContactConstraintSolver.GetStats(); }

		void SetBroadphaseNodeBoundThreshold(float treshold) { mBroadphase->SetBoundThreshold(treshold); }
		void SetFrictionCombineMode(ECombineMode friction_combine_mode) 
		{ 
			mSettings.solver.frictionCombineMode = friction_combine_mode;
			mContactConstraintSolver.SetFrictionCombineMode(friction_combine_mode); 
		}
		void SetRestitutionCombineMode(ECombineMode restitution_combine_mode) 
		{
			mContactConstraintSolver.SetRestitutionCombineMode(mSettings.solver.restitutionCombineMode); 
			mSettings.solver.restitutionCombineMode = restitution_combine_mode;
		}

		//TODO(Jay): Later dont return broadphase only state
		VX_INLINE Broadphase* GetBroadphase() { return mBroadphase; }
		//VX_INLINE BVHBroadphase<AABB>* GetBVH_AABB_Broadphase() { return mBroadphase->AsBVH_AABB(); }
		const BroadphasePair* GetBroadphasePairsPtr() const { return mBroadphaseBuffer.data; }
		const uint32 GetBroadphasePairsCount() const { return mBroadphaseBuffer.count; }


		const BodyManager& GetBodyManager() const { return mBodyManager; }
		BodyManager& GetBodyManager() { return mBodyManager; }

		static uint32 GetCurrentSimStep() { return static_cast<uint32>(mFrameIdx); }


		WorldQuery GetWorldQuery() const { return mWorldQuery; }


		BodyID* GetActiveBodies() const { return mActiveBodies; }
		uint32 GetNumActiveBodies() const { return mNumActiveBodies; }
	private:
		PhysicsWorldSettings mSettings;
		PhysicsStepContext mContext;
		//std::vector<Body> mBodies;

		BodyManager mBodyManager;

		uint32 mMaxActiveBodies = 256;
		BodyID* mActiveBodies = nullptr;
		uint32 mNumActiveBodies = 0;
		void UpdateBodiesActivationState(float dt);


		Broadphase* mBroadphase = nullptr;
		//std::vector<BroadphasePair> mBroadphasePairs;
		//BroadphasePair* mBroadphasePairs;
		//uint32 mBroadpairCount = 0;

		struct BroadphaseBuffer
		{
			BroadphasePair* data;
			uint32 count = 0;
			uint32 maxPairs = 0;
		}mBroadphaseBuffer;
		std::vector<class ContactManifold> mStepManifolds;
		NarrowphaseQuery mNarrowphaseQuery;

		ContactConstraintSolver mContactConstraintSolver;

		ConstraintCoordinator mConstraintCoordinator;
		class ConstraintSolver* mConstraintSolver = nullptr;

		//hack for now 
		DebugGizmosRenderer* mHackDebugRenderer = nullptr;

		static uint64 mFrameIdx;

		Colour GetBodySimphaseDebugColour(const Body& body) const;

		WorldQuery mWorldQuery;
		void* testAllocation;
		size_t testAllocationSize;
	public:
		class ScratchAllocator* mScratchAllocator = nullptr;
	};
}