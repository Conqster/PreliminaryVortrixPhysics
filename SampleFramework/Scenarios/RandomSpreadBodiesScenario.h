#pragma once
#include "Scenario.h"


class RandomSpreadBodiesScenario : public Scenario
{
public:

	void Init(vx::PhysicsWorld* i_world) override
	{
		Scenario::Init(i_world);
		CreateGroundPlane(200.0f);
	}
	const char* Name() override { return "Empty Scenario"; }
	const char* Info() override
	{
		return "Empty scenerio with ground plane; to create & interact with";
	}


	void GenerateBodies(vx::RefConst<vx::Shape> shape, float spawn_radius);
};




class RandomSpreadBoxBodiesScenario : public RandomSpreadBodiesScenario
{
public:

	void Init(vx::PhysicsWorld* i_world) override;
	const char* Name() override { return "Random Spread Box Bodies Scenario"; }
	const char* Info() override
	{
		return "Empty scenerio with ground plane; to create & interact with";
	}
};