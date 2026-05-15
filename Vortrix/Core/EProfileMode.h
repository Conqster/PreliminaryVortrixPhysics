#pragma once

#include "Core.h"


namespace vx
{
	namespace Profiler
	{
		enum class EProfileMode : uint8_t
		{
			None			= 0,
			Deterministic	= Bit8(0),
			VariableRate	= Bit8(1),
			All				= Deterministic | VariableRate
		};


		inline EProfileMode operator|(EProfileMode lhs, EProfileMode rhs)
		{
			using underlying_t = std::underlying_type_t<EProfileMode>;
			return static_cast<EProfileMode>(static_cast<underlying_t>(lhs) | static_cast<underlying_t>(rhs));
		}


		inline EProfileMode operator&(EProfileMode lhs, EProfileMode rhs)
		{
			using underlying_t = std::underlying_type_t<EProfileMode>;
			return static_cast<EProfileMode>(static_cast<underlying_t>(lhs) & static_cast<underlying_t>(rhs));
		}

		inline bool Contains(const EProfileMode& bits, const EProfileMode& bit)
		{
			using underlying_t = std::underlying_type_t<EProfileMode>;
			return static_cast<underlying_t>(bits & bit) != 0;
		}

		inline bool Any(const EProfileMode& bits)
		{
			using underlying_t = std::underlying_type_t<EProfileMode>;
			return static_cast<uint8_t>(bits) != 0;
		}
	}
}