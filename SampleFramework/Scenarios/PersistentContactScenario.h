#pragma once
#include "Scenario.h"


class PersistentContactScenario : public Scenario
{
public:

	void Init(vx::PhysicsWorld* i_world) override;
	const char* Name() override { return "Persistent Contact Scenario"; }
	const char* Info() override
	{
		return "Scenario to test persistent Contact Scenario";
	}

	void OnUI() override;
};