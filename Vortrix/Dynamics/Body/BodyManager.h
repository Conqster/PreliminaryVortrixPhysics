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
	class BodyManager
	{
	public:
		void Init(uint32 max_bodies);

		~BodyManager();

		bool AddBody(const BodySettings& body_setting);
		bool AddBody(Body& _body);

		std::vector<Body>& GetBodies() { return mBodies; }
		std::vector<BodyDebug>& GetBodiesDebug() { return mBodiesDebugInfo; }

		const Body& GetBody(BodyID id) const { return mBodies[id.Value()]; }
		Body& GetBody(BodyID id) { return mBodies[id.Value()]; }

		const BodyDebug& GetBodyDebugInfo(const Body& body) const;
		const BodyDebug& GetBodyDebugInfo(BodyID id) const { return mBodiesDebugInfo[id.Value()]; }
		BodyDebug& GetBodyDebugInfo(const Body& body);
		BodyDebug& GetBodyDebugInfo(BodyID id) { return mBodiesDebugInfo[id.Value()]; }

		const char* GetBodyDebugName(const Body& body) const;
		const char* GetBodyDebugName(BodyID id) const { return mBodiesDebugInfo[id.Value()].name.Data(); }

		const BodySimStats& GetBodySimStats(const Body& body) const;
		BodySimStats& GetBodySimStats(const Body& body);
		const BodySimStats& GetBodySimStats(BodyID id) const { return mBodiesDebugInfo[id.Value()].simulationStats; }

		void UpdateBodyVelocitySimStat(const Body& body);

		VX_INLINE uint32 MaxBodies() const { return mMaxBodies; }

	private:
		std::vector<Body> mBodies;
		std::vector<BodyDebug> mBodiesDebugInfo;

		uint32 mMaxBodies = 0;
	};

}