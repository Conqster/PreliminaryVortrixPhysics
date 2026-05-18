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

////Things to do 
/// Implement 
///		MotionDynamics
/// fix shapes; especially box; function needs to be in local space
/// 
/// And in Release ensure some debug prints are striped out
/// 
/// play around the opportunity to use a list for active bodies. 
/// 
/// 
/// 
/// a deadly bug, you can just set the desnsity of a shap ein body manage 
/// the shape might be shared between multiple bodies
/// 
/// 
/// Texture Factory Add Texture to registy not self
/// 
/// flags option
/// VX_DISABLE_FORCE_INLINE
/// VX_DEBUG
/// 
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
		~PhysicsWorld()
		{
			//mBodies.clear();
			delete[] mActiveBodies;

			delete mBroadphase;
			delete mNarrowphaseQuery;
		}

		static void CreateSimpleWorld(PhysicsWorld* io_world);

		static void GenerateWorldDefaultConfig(int& o_max_bodies, int& o_max_body_pairs, int& o_max_contact_constraint);
		void Init(float max_bodies, float max_body_pairs, float max_contact_constraint);

		void CreateBody(const BodySettings& body_setting);
		void AddBody(const Body& body);

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

		class Joint* mTestJoint = nullptr;

		void QuickDebugDrawInertia(DebugGizmosRenderer* debug_renderer);

		std::vector<Body>& GetBodies() { return mBodyManager.GetBodies(); }

		const PhysicsWorldSettings& GetSettings() const { return mSettings; }
		PhysicsWorldSettings& GetSettings() { return mSettings; }

		const struct CollisionResolutionStat* GetNarrowphaseStats() const;

		const ContactConstraintSolver::ContactConstraintSolverStat& GetContactConstraintSolverStats() const 
		{ return mContactConstraintSolver.GetStats(); }

		//TODO(Jay): Later dont return broadphase only state
		VX_INLINE Broadphase* GetBroadphase() { return mBroadphase; }
		//VX_INLINE BVHBroadphase<AABB>* GetBVH_AABB_Broadphase() { return mBroadphase->AsBVH_AABB(); }
		const std::vector<BroadphasePair>& GetBroadphasePairs() const { return mBroadphasePairs; }

		///hack to convert colour32 to glm
		//VX_INLINE static glm::vec3 ColourToGLM(const Colour& col)
		//{
		//	return glm::vec3(static_cast<float>(col.r), static_cast<float>(col.g), static_cast<float>(col.b)) / 255.0f;
		//}

		void QuickBoxBoxDebug(DebugGizmosRenderer* debug_renderer);

		const BodyManager& GetBodyManager() const { return mBodyManager; }
		BodyManager& GetBodyManager() { return mBodyManager; }

		static uint32 GetCurrentSimStep() { return static_cast<uint32>(mFrameIdx); }

		void UpdateSystem();

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
		std::vector<BroadphasePair> mBroadphasePairs;
		std::vector<class ContactManifold> mStepManifolds;
		NarrowphaseQuery* mNarrowphaseQuery = nullptr;

		ContactConstraintSolver mContactConstraintSolver;

		//hack for now 
		DebugGizmosRenderer* mHackDebugRenderer = nullptr;

		static uint64 mFrameIdx;

		Colour GetBodySimphaseDebugColour(const Body& body) const;


		std::array<Colour, 32> mRandomColourInst;

		WorldQuery mWorldQuery;

	public:
		//later convert this to a single flag
		bool mCollsionSettingDirty = false;
		bool mSolverSettingDirty = false;
	};
}