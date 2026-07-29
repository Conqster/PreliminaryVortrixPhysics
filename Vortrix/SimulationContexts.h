#pragma once

#include "Vortrix/PhysicsWorldSettings.h"

namespace vx {
	struct CollisionContext
	{
		const CollisionSettings& settings;
		DebugGizmosRenderer* debugRenderer = nullptr;
		bool drawContactTBNs = false;
		uint32 physicsFrameIdx = 0;

		uint32 broadphasePairCount = 0;

		//might become island builder/coordinator
		class ConstraintSolver* constraintSolver = nullptr;
		class IslandCoordinator* islandCoord = nullptr;
	};
}