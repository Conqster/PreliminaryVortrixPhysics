#pragma once

#include "Core/Core.h"

namespace vx
{

	//later  shader by stage /phase
	enum class EBodySimphaseFlags : uint8
	{
		None			= 0,
		InBroadphase	= Bit8(0),    /// in broadphase
		InNarrowphase	= Bit8(1),    /// paired and participate in narrowphase
		IsColliding		= Bit8(2),	  /// colliding a body 
		IsTouchingStatic= Bit8(3),    /// initiate collision with non moving body
		//IsIslandRoot = Bit8(4)
	};


	inline EBodySimphaseFlags operator|(EBodySimphaseFlags lhs, EBodySimphaseFlags rhs)
	{
		using underlying_t = std::underlying_type_t<EBodySimphaseFlags>;
		return static_cast<EBodySimphaseFlags>(static_cast<underlying_t>(lhs) | static_cast<underlying_t>(rhs));
	}

	inline EBodySimphaseFlags operator|=(EBodySimphaseFlags& lhs, EBodySimphaseFlags rhs)
	{
		lhs = lhs | rhs;
		return lhs;
	}

	inline EBodySimphaseFlags operator&(EBodySimphaseFlags lhs, EBodySimphaseFlags rhs)
	{
		using underlying_t = std::underlying_type_t<EBodySimphaseFlags>;
		return static_cast<EBodySimphaseFlags>(static_cast<underlying_t>(lhs) & static_cast<underlying_t>(rhs));
	}

	inline bool Contains(EBodySimphaseFlags& bits, const EBodySimphaseFlags& bit)
	{
		return (static_cast<uint8>(bits) & static_cast<uint8>(bit));
	}

} // vx namespa