#include "BodyManager.h"

#include "Body.h"
#include "BodySimStats.h"

#include "Collision/Shapes/Shape.h"


namespace vx
{
	void BodyManager::Init(uint32 max_bodies)
	{
		mMaxBodies = max_bodies;
		mBodies.reserve(mMaxBodies);
		mBodiesDebugInfo.reserve(mMaxBodies);
	}
	BodyManager::~BodyManager()
	{
		//for (auto& b : mBodies)
		//	delete b.mShape;
	}
	bool BodyManager::AddBody(const BodySettings& body_setting)
	{
		if(mBodies.size() >= mMaxBodies - 1)
		{
			VX_WARN("Body manager body limit attained");
			return false;
		}

		vx::Body body = Body(body_setting.position);

		body.mOrientation = body_setting.orientation;
		body.mFriction = body_setting.friction;
		body.mRestitution = body_setting.restitution;
		body.mMotionType = body_setting.motionType;

		float density = 0.0f;

		if (body_setting.motionType == EMotionType::Dynamic)
		{
			if (body_setting.mass > 0.0f) //to support deprecated method
				density = body_setting.density;
		
			body.mLinearVelocity = Vec3(0.0f);
			body.mAngularVelocity = Vec3(0.0f);

			body.mLinearDamping = body_setting.linearDamping;
			body.mAngularDamping = body_setting.angularDamping;
			body.mAwake = true;
			body.mSleepTimer = 0.0f;
		
			body.ClearAccumulatedForces();

			body.mMaxLinearVelocity = body_setting.maxLinearVelocity;		
			body.mMaxAngularVelocity = body_setting.maxAngularVelocity;

			body.mAllowedDynamicsDof = body_setting.degreeFreedom;
		}
		else
			body.mAwake = false;
		

		body_setting.shape->SetDensity(density);
		/// Set shape handles the bodies shape 
		/// as well as bounds 
		body.SetShape(body_setting.shape);

		/// Post body setup 
		if (!body_setting.intialVelocity.IsZero())
			body.SetLinearVelocity(body_setting.intialVelocity);
		if (!body_setting.impluse.IsZero())
			body.ApplyImpulse(body_setting.impluse);

		//since orientation might have change percompute bounds
		body.ComputeWorldSpaceBoundsInternal();

		bool success = AddBody(body);
		success &= body.GetID().Value() == mBodiesDebugInfo.size();

		//new body debug 
		VX_ASSERT_WARN(success, "Invalid Body creation or Miss-matching id for body & body debug");
		BodyDebug& body_debug = mBodiesDebugInfo.emplace_back();
		StackString<40> _s(body_setting.debug_name);
		_s << " - body " << body.GetID().Value();
		//body_debug.name = body_setting.debug_name + " - body " + std::to_string(body.GetID().Value());
		body_debug.name = _s;
		return success;
	}
	bool BodyManager::AddBody(Body& _body)
	{
		if (_body.GetID().IsValid())
			return false;

		uint32 idx;

		idx = static_cast<uint32>(mBodies.size());
		_body.mID = BodyID(idx);

		mBodies.emplace_back(_body);

		return true;
	}

	const BodyDebug& BodyManager::GetBodyDebugInfo(const Body& body) const
	{
		return GetBodyDebugInfo(body.GetID());
	}

	BodyDebug& BodyManager::GetBodyDebugInfo(const Body& body)
	{
		return GetBodyDebugInfo(body.GetID());
	}

	const char* BodyManager::GetBodyDebugName(const Body& body) const
	{
		return GetBodyDebugName(body.GetID());
	}

	const BodySimStats& BodyManager::GetBodySimStats(const Body& body) const
	{
		return GetBodySimStats(body.GetID());
	}

	BodySimStats& BodyManager::GetBodySimStats(const Body& body)
	{
		return mBodiesDebugInfo[body.GetID().Value()].simulationStats;
	}

	void BodyManager::UpdateBodyVelocitySimStat(const Body& body)
	{
		BodySimStats& sim_stat = mBodiesDebugInfo[body.GetID().Value()].simulationStats;

		sim_stat.maxAttainedLinearVelocitySq = VxMax(sim_stat.maxAttainedLinearVelocitySq, body.GetLinearVelocity().LengthSq());
		sim_stat.maxAttainedAngularVelocitySq = VxMax(sim_stat.maxAttainedAngularVelocitySq, body.GetAngularVelocity().LengthSq());
	}

}