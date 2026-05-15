#pragma once
#include "Vortrix/Core/Core.h"
#include "Vortrix/Maths/Vec3.h"

namespace vx {


	enum class EDynamicsDofs : uint8
	{
		//None = 0,
		TranslateX = Bit8(0),
		TranslateY = Bit8(1),
		TranslateZ = Bit8(2),

		RotationX = Bit8(3),
		RotationY = Bit8(4),
		RotationZ = Bit8(5),

		All = TranslateX |TranslateY | TranslateZ |
				RotationX |RotationY |RotationZ,

		None = !All,
	};




	VX_INLINE bool Contains(EDynamicsDofs dof, EDynamicsDofs flag)
	{
		return (uint8(dof) & uint8(flag)) != 0;
	}

	VX_INLINE Vec3 AllowedLinearDofsMask(const EDynamicsDofs& dof)
	{
		float x = float(Contains(dof, EDynamicsDofs::TranslateX));
		float y = float(Contains(dof, EDynamicsDofs::TranslateY));
		float z = float(Contains(dof, EDynamicsDofs::TranslateZ));
		return Vec3(x, y, z);
	}

	VX_INLINE Vec3 AllowedAngularDofsMask(const EDynamicsDofs& dof)
	{
		float x = float(Contains(dof, EDynamicsDofs::RotationX));
		float y = float(Contains(dof, EDynamicsDofs::RotationY));
		float z = float(Contains(dof, EDynamicsDofs::RotationZ));
		return Vec3(x, y, z);
	}
}