#pragma once
#include "Scenario.h"

#include "Dynamics/Joint.h"

class JointScenario : public Scenario
{
public:
	void Init(vx::PhysicsWorld* i_world) override;
	const char* Name() override { return "Joint Scenario"; }
	const char* Info() override
	{
		return "Experiment with Joint constraints, etc";
	}

	void PostPhysicsStep(float dt) override;
	void OnUI() override;

private: 
	//DistanceConstraint mJoint;
	//std::vector<DistanceConstraint> mNotInPipelineJoints;
};