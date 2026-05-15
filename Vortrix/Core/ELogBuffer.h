#pragma once


#include "Core.h"

namespace vx
{
	//write to either / both the 
	//console buffer sink 
	//a file output 
	enum class ELogBuffer : uint8_t
	{
		None	= 0,
		Console = Bit8(0),
		File	= Bit8(1)
	};


	inline ELogBuffer operator|(ELogBuffer lhs, ELogBuffer rhs)
	{
		using underlying_t = std::underlying_type_t<ELogBuffer>;
		return static_cast<ELogBuffer>(static_cast<underlying_t>(lhs) | static_cast<underlying_t>(rhs));
	}

	inline ELogBuffer operator|=(ELogBuffer& lhs, ELogBuffer rhs)
	{
		lhs = lhs | rhs;
		return lhs;
	}

	inline ELogBuffer operator&(ELogBuffer lhs, ELogBuffer rhs)
	{
		using underlying_t = std::underlying_type_t<ELogBuffer>;
		return static_cast<ELogBuffer>(static_cast<underlying_t>(lhs) & static_cast<underlying_t>(rhs));
	}

	inline bool Contains(const ELogBuffer& bits, const ELogBuffer& bit)
	{
		using underlying_t = std::underlying_type_t<ELogBuffer>;
		return static_cast<underlying_t>(bits & bit) != 0;
	}
} // VPHX namespace