#pragma once

#include "Vortrix/Core/Core.h"

enum class ETextAlignment : vx::uint8
{
	Left,
	Center,
	Right
};
constexpr const char* kTextAlignmentModelNames = "Left\0""Center\0""Right\0""\0";
