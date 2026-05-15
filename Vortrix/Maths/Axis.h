#pragma once
#include "MathCommons.h"

namespace vx
{
	enum class Axis : uint8
	{
		X = 0, 
		Y = 1,
		Z = 2,
		W = 3
	};

	//constant aliases
	constexpr Axis kAxisX = Axis::X;
	constexpr Axis kAxisY = Axis::Y;
	constexpr Axis kAxisZ = Axis::Z;
	constexpr Axis kAxisW = Axis::W;
}