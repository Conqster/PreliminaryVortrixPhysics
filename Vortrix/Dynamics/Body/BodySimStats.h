#pragma once


#include "EBodyDebugFlags.h"

namespace vx
{
	struct BodySimStats
	{
		void Reset()
		{
			phase = EBodySimphaseFlags::None;
		}

		/// for the debug phases body passes throught 
		EBodySimphaseFlags phase = EBodySimphaseFlags::None;
		float maxAttainedLinearVelocitySq = 0.0f;
		float maxAttainedAngularVelocitySq = 0.0f;
	};
}