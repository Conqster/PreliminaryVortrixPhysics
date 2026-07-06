#pragma once

#include "Vortrix/Core/Core.h"

namespace vx {

	enum class EConstraintFlags : uint8
	{
		None = 0,
		Active = Bit8(0),    
		SolveVelocity = Bit8(1),    
		SolvePosition = Bit8(2)
	};


	inline EConstraintFlags operator|(EConstraintFlags lhs, EConstraintFlags rhs)
	{
		using T = std::underlying_type_t<EConstraintFlags>;
		return static_cast<EConstraintFlags>(static_cast<T>(lhs) | static_cast<T>(rhs));
	}

	inline EConstraintFlags& operator|=(EConstraintFlags& lhs, EConstraintFlags rhs)
	{
		lhs = lhs | rhs;
		return lhs;
	}

	inline EConstraintFlags operator&(EConstraintFlags lhs, EConstraintFlags rhs)
	{
		using T = std::underlying_type_t<EConstraintFlags>;
		return static_cast<EConstraintFlags>(static_cast<T>(lhs) & static_cast<T>(rhs));
	}

	inline EConstraintFlags& operator&=(EConstraintFlags& lhs, EConstraintFlags rhs)
	{
		lhs = lhs & rhs;
		return lhs;
	}

	inline EConstraintFlags operator~(EConstraintFlags value)
	{
		using T = std::underlying_type_t<EConstraintFlags>;
		return static_cast<EConstraintFlags>(~static_cast<T>(value));
	}

	inline bool Contains(EConstraintFlags value, EConstraintFlags flags)
	{
		using T = std::underlying_type_t<EConstraintFlags>;
		return (static_cast<T>(value) & static_cast<uint8>(flags)) != 0;
	}

} // namespace vx