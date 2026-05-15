#pragma once

#include "ERenderInstanceFlags.h"
#include "ETextAlignment.h"

namespace vx {
	struct RenderSettings
	{
		ERenderInstanceFlags sphereInstanceFlags = ERenderInstanceFlags::CastShadow | ERenderInstanceFlags::ReceiveShadow | ERenderInstanceFlags::UseTexture;
		ERenderInstanceFlags boxInstanceFlags = ERenderInstanceFlags::CastShadow | ERenderInstanceFlags::ReceiveShadow;
		ERenderInstanceFlags planeInstanceFlags = ERenderInstanceFlags::ReceiveShadow;
		ERenderInstanceFlags capsuleInstanceFlags = ERenderInstanceFlags::CastShadow | ERenderInstanceFlags::ReceiveShadow;
	
		ETextAlignment textAlignment = ETextAlignment::Center;
		float textScale = 0.0003f;
		bool useTestDynamicScale = true;

		bool drawWorldAxes = true;
		float worldAxesLength = 35.0f;
	};
}
