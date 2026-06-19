#pragma once

#include <string>
#include <vector>
#include "BodyID.h"
#include "BodyDebug.h"
#include "Body.h"
namespace vx 
{
	class Body;
	class BodySettings;

	using BodyVector = std::vector<Body>;
	
	class BodyManager
	{
	public:
		void Init(uint32 max_bodies);

		~BodyManager();

		const BodyID AddBody(const BodySettings& body_setting);
		const BodyID AddBody(Body& _body);

		template<EMotionType Type>
		void SetBodyMotionType(Body& body, bool update_mass_inertia);
		void SetBodyShape(Body& body, RefConst<Shape> shape, bool update_mass_inertia);

		//it bettre to remove body via Physocs world as to manage broadphase handles etc
		void RemoveBody(const Body& body) { RemoveBody(body.GetID()); }
		void RemoveBody(const BodyID& id);

		BodyVector& GetBodies() { return mBodies; }
		std::vector<BodyDebug>& GetBodiesDebug() { return mBodiesDebugInfo; }

		const Body& GetBody(BodyID id) const { return mBodies[id.Idx()]; }
		Body& GetBody(BodyID id) { return mBodies[id.Idx()]; }

		const BodyDebug& GetBodyDebugInfo(const Body& body) const;
		const BodyDebug& GetBodyDebugInfo(BodyID id) const { return mBodiesDebugInfo[id.Idx()]; }
		BodyDebug& GetBodyDebugInfo(const Body& body);
		BodyDebug& GetBodyDebugInfo(BodyID id) { return mBodiesDebugInfo[id.Idx()]; }

		const char* GetBodyDebugName(const Body& body) const;
		const char* GetBodyDebugName(BodyID id) const { return mBodiesDebugInfo[id.Idx()].name.Data(); }

		const BodySimStats& GetBodySimStats(const Body& body) const;
		BodySimStats& GetBodySimStats(const Body& body);
		VX_INLINE const BodySimStats& GetBodySimStats(BodyID id) const { return mBodiesDebugInfo[id.Idx()].simulationStats; }
		VX_INLINE BodySimStats& GetBodySimStats(BodyID id) { return mBodiesDebugInfo[id.Idx()].simulationStats; }

		void UpdateBodyVelocitySimStat(const Body& body);

		VX_INLINE uint32 MaxBodies() const { return mMaxBodies; }

	private:
		BodyVector mBodies;
		std::vector<BodyDebug> mBodiesDebugInfo;

		uint32 mMaxBodies = 0;


		std::vector<uint32> mFreedIdxs;
		std::vector<uint8> mBodyIdxGenerations;

		VX_INLINE uint8 GetBodyIdxNextGeneration(uint32 idx)
		{
			return ++mBodyIdxGenerations[idx];
		}
	};

}