#pragma once
#include "Scenario.h"


class BoxStackScenario : public Scenario
{
public:
	void Init(vx::PhysicsWorld* i_world) override;
	const char* Name() override { return "Box Stack Scenario"; }
	const char* Info() override
	{
		return "Test bodies resitituion cofficent";
	}
};