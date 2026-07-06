#pragma once

#include "BodySimStats.h"
#include "Vortrix/Core/StackString.h"
#include "BodyID.h"
namespace vx{
	/// the goal is to remove debug info, 
	/// whihch is a cold data compared to the Body class
	/// at having string in body which will hurt cache locality 
	struct BodyDebug
	{
		//std::string name;
		BodySimStats simulationStats;							//12 bytes	[16 bytes]
		StackString<40> name;
	};
	static_assert(sizeof(BodyDebug) == 64);
}
