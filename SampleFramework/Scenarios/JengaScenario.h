#pragma once


#pragma once
#include "Scenario.h"


class JengaScenario : public Scenario
{
public:
	void Init(vx::PhysicsWorld* i_world) override;
	const char* Name() override { return "Jenga Scenario"; }
	const char* Info() override
	{
		return "Test bodies resitituion cofficent";
	}
};