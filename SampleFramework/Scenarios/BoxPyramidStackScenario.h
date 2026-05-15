#pragma once
#include "Scenario.h"


class BoxPyramidStackScenario : public Scenario
{
public:
	void Init(vx::PhysicsWorld* i_world) override;
	const char* Name() override { return "Box Pyramid Stack Scenario"; }
	const char* Info() override
	{
		return "Test bodies resitituion cofficent";
	}


};