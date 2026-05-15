#pragma once
#include "Scenario.h"


class SimpleBasicScenario : public Scenario
{
public:

	void Init(vx::PhysicsWorld* i_world) override;
	const char* Name() override { return "Simple Basic Scenario"; }
	const char* Info() override 
	{
		return "Default Simple scenerio; to interact with";
	}
};