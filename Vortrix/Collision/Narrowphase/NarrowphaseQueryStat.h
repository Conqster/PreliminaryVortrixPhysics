#pragma once

#include "Vortrix/Vortrix.h"

namespace vx {
	struct CollisionResolutionStat
	{
		uint32 numPairReceived = 0;
		uint32 numContactPair = 0;

		uint32 maxAttainedContactPair = 0;
	};
}