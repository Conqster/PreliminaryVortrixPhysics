#pragma once
#include "Scenario.h"

namespace vx {
	class Constraint;
}

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
	void PostPhysicsStep(float dt) override;

	void OnClose() override;
	~PersistentContactScenario();

private:
	vx::Constraint* mTestConstraint;
};