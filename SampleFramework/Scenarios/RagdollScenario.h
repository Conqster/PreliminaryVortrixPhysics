#pragma once
#include "Scenario.h"


class RagdollScenario : public Scenario
{
public:
	void Init(vx::PhysicsWorld* i_world) override;
	const char* Name() override { return "Ragdoll Scenario"; }
	const char* Info() override
	{
		return "Experiment with Ragdoll constraints, etc";
	}

	void PostPhysicsStep(float dt) override;
	void OnUI() override;
};