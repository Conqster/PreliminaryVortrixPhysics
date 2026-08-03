#include "Scenario.h"


class EmptyScenario : public Scenario
{
public:

	void Init(vx::PhysicsWorld* i_world) override
	{
		Scenario::Init(i_world);
		CreateGroundPlane(100.0f);
	}
	const char* Name() override { return "Empty Scenario"; }
	const char* Info() override
	{
		return "Empty scenerio with ground plane; to create & interact with";
	}
};