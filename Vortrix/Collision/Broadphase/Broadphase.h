#pragma once

#include "Vortrix/Vortrix.h"
#include "Vortrix/Collision/WorldQueryContext.h"


namespace vx
{

#if defined(VX_PROFILE_BROAD)
	enum class EBroadphaseType : uint8
	{
		Bruteforce,
		BVH

		//more broadphase types 
		//BVH4, Quad etc
	};	
#endif // defined(VX_PROFILE_BROAD)

	struct BroadphaseInitInfo
	{
		uint32 maxDirtyBodies = 128;
	};



	//struct RayShapeDispatch;
	struct RayCast;
	using WorldRayCastQuery = WorldQueryContext<RayCast, RayShapeFn>;

	class Body;
	struct DrawSettings;

	class DebugGizmosRenderer;

	class Broadphase
	{
	public:
		virtual ~Broadphase() = default;
		virtual void Init(class BodyManager* in_body_manager, const BroadphaseInitInfo& info) = 0;


		virtual void InsertBody(Body* body) = 0;
		virtual void RemoveBody(const BodyID& id) = 0;
		virtual void ComputeCollidingPair(const struct PhysicsStepContext& physics_ctx, struct SimStep& io_sim_step) = 0;
		virtual void DebugDraw(DebugGizmosRenderer*, const DrawSettings&) = 0;

		virtual void SetBoundThreshold(float v) { mBoundThreshold = v;}

		virtual void CastRay(const RayCast& ray_cast, WorldRayCastQuery& world_ray_ctx) const = 0;


#if defined(VX_PROFILE_BROAD)
		virtual EBroadphaseType Type() const = 0;// <-- abstract to unsure that all inherited class defines it
#endif // defined(VX_PROFILE_BROAD)
	protected: 
		float mBoundThreshold = 0.2f;
		uint32 mLastStep = 0;
	private:
	};
}