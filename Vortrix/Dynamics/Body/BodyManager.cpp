#include "BodyManager.h"

#include "Body.h"
#include "BodySimStats.h"

#include "Collision/Shapes/Shape.h"

#include <set>
namespace vx
{
	void BodyManager::Init(uint32 max_bodies)
	{
		mMaxBodies = max_bodies;
		mBodies.reserve(mMaxBodies);
		mBodiesDebugInfo.reserve(mMaxBodies);

		uint32 min_free_list = 64;
		mFreedIdxs.reserve(min_free_list);
		mBodyIdxGenerations.resize(mMaxBodies, 0);
	}
	BodyManager::~BodyManager()
	{
		/// JAY: move to smart pointer; ref counted
		/// solve with set for now 
		/// because multiple bodies might be sharing 
		/// a shape
		//std::set<Shape*> shapes;
		//for (auto& b : mBodies)
		//	shapes.insert(b.mShape);
		//
		//for(auto& s : shapes)
		//		delete s;
	}
	const BodyID BodyManager::AddBody(const BodySettings& body_setting)
	{
		if(mBodies.size() >= mMaxBodies - 1 && mFreedIdxs.empty())
		{
			VX_LOG_WARN("Body manager body limit attained");
			return BodyID();
		}

		vx::Body body = Body(body_setting.position);

		body.mOrientation = body_setting.orientation;
		body.mFriction = body_setting.friction;
		body.mRestitution = body_setting.restitution;
		body.mMotionType = body_setting.motionType;


		if (body_setting.motionType == EMotionType::Dynamic)
		{
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

		BodyID id = AddBody(body);
		bool success = id.IsValid();


		if (id.Generation() <= 1U)
			mBodiesDebugInfo.emplace_back();

		success &= body.GetID().Idx() < mBodiesDebugInfo.size();
		VX_ASSERT_WARN(success, "Invalid Body creation or Miss-matching id for body & body debug");
		StackString<40> _s(body_setting.debug_name);
		_s << " - body_" << body.GetID().ID() << ", idx_" << body.GetID().Idx();
		mBodiesDebugInfo[body.GetID().Idx()].name = _s;
		return id;
	}
	const BodyID BodyManager::AddBody(Body& _body)
	{
		if (_body.GetID().IsValid())
			return _body.GetID();


		BodyID id = BodyID();
		if (!mFreedIdxs.empty())
		{
			uint32 idx = mFreedIdxs[0];
			std::swap(mFreedIdxs[0], mFreedIdxs.back());
			mFreedIdxs.pop_back();
			
			uint8 gen = GetBodyIdxNextGeneration(idx);
			id = BodyID(idx, gen);
			_body.mID = id;
			mBodies[idx] = _body;
		}
		else
		{
			uint32 idx = static_cast<uint32>(mBodies.size());

			uint8 gen = GetBodyIdxNextGeneration(idx);
			id = BodyID(idx, gen);
			_body.mID = id;
			mBodies.emplace_back(_body);
		}
		return id;
	}

	void BodyManager::RemoveBody(const BodyID& id)
	{
		VX_ASSERT(id.IsValid(), "Attempting to remoev invalid body");


		//remove from broadphase 

		mFreedIdxs.push_back(id.Idx());
		mBodies[id.Idx()].mID = BodyID();
		//this would affect shape 

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
		return GetBodySimStats(body.GetID());
		//return mBodiesDebugInfo[body.GetID().Value()].simulationStats;
	}

	void BodyManager::UpdateBodyVelocitySimStat(const Body& body)
	{
		//BodySimStats& sim_stat = mBodiesDebugInfo[body.GetID().Value()].simulationStats;
		
		BodySimStats& sim_stat = GetBodySimStats(body.GetID());

		sim_stat.maxAttainedLinearVelocitySq = VxMax(sim_stat.maxAttainedLinearVelocitySq, body.GetLinearVelocity().LengthSq());
		sim_stat.maxAttainedAngularVelocitySq = VxMax(sim_stat.maxAttainedAngularVelocitySq, body.GetAngularVelocity().LengthSq());
	}

}