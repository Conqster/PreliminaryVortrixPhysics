#pragma once

#include "Vortrix/PhysicsWorldSettings.h"

namespace vx {
	struct CollisionContext
	{
		const CollisionSettings& settings;
		DebugGizmosRenderer* debugRenderer = nullptr;
		bool drawContactTBNs = false;
		uint32 physicsFrameIdx = 0;
	};
}