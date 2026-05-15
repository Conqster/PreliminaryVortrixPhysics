#pragma once 
#include "Scenario.h"


class FrictionScenario : public Scenario
{
public: 	
	explicit FrictionScenario(float slop_angle) : mSlopAngle(slop_angle){}
	void Init(vx::PhysicsWorld*) override;

	//const char* Name() override { return "Friction Scenario"; }
	const char* Info() override
	 {
	  return "Test bodies friction cofficent";
	  }

protected:
	float mSlopAngle = 22.5f;
};


class FrictionScenario22_5 : public FrictionScenario
{
public:
	FrictionScenario22_5() : FrictionScenario(22.5f) {}

	void Init(vx::PhysicsWorld* i_world) override;
	const char* Name() override { return "Friction Scenario 22.5 Degrees"; }
};

class FrictionScenario45 : public FrictionScenario
{
public:
	void Init(vx::PhysicsWorld* i_world) override;
	FrictionScenario45() : FrictionScenario(45.0f) {}
	const char* Name() override { return "Friction Scenario 45 Degrees"; }
};

