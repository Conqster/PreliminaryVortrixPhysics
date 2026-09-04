#pragma once

#include "Vortrix/Core/Core.h"

namespace vx {
	/// shadow cast test (flags & 1) != 0
	/// receive shadow test (flags & 2) != 0 etc
	/// 
	/// easy add 
	/// flags |= CastShadow
	/// remove 
	/// flags &= ~CastShadow
	/// toggle (kind of emissive flash)
	/// flags ^= Emission 
	enum class ERenderInstanceFlags : uint32
	{
		None = 0,
		CastShadow = Bit32(0), // 1 (0001) vx::Bit(0),
		ReceiveShadow = Bit32(1), //2(0010)
		Emissive = Bit32(2), //4 (0100)

		UseTexture = Bit32(3),
		Wireframe = Bit32(4)
	};

	constexpr const char* RenderInstanceFlagsNames = "Cast Shadow\0""Recieve Shadow\0""Emissive\0""\0";

	//add flags OR
	constexpr ERenderInstanceFlags operator |(ERenderInstanceFlags lhs, ERenderInstanceFlags rhs)
	{
		return ERenderInstanceFlags(vx::uint32(lhs) | vx::uint32(rhs));
	}

	constexpr ERenderInstanceFlags& operator |=(ERenderInstanceFlags& io_lhs, ERenderInstanceFlags rhs)
	{
		io_lhs = io_lhs | rhs;
		return io_lhs;
	}

	//to remove a flag NOT
	constexpr ERenderInstanceFlags operator ~(ERenderInstanceFlags flag) { return ERenderInstanceFlags(~vx::uint32(flag)); }
	constexpr ERenderInstanceFlags operator & (ERenderInstanceFlags lhs, ERenderInstanceFlags rhs)
	{
		return ERenderInstanceFlags(vx::uint32(lhs) & vx::uint32(rhs));
	}
	constexpr ERenderInstanceFlags& operator &= (ERenderInstanceFlags& io_lhs, ERenderInstanceFlags rhs)
	{
		io_lhs = io_lhs & rhs;
		return io_lhs;
	}

	//to toggle XOR
	constexpr ERenderInstanceFlags operator ^(ERenderInstanceFlags lhs, ERenderInstanceFlags rhs)
	{
		return ERenderInstanceFlags(vx::uint32(lhs) ^ vx::uint32(rhs));
	}

	constexpr ERenderInstanceFlags& operator ^= (ERenderInstanceFlags& io_lhs, ERenderInstanceFlags rhs)
	{
		io_lhs = io_lhs ^ rhs;
		return io_lhs;
	}

}
