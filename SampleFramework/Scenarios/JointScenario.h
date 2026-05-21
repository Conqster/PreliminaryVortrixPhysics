#pragma once
#include "Scenario.h"


namespace vx {
	class DistanceConstraint;
} //namespace vx

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
	vx::DistanceConstraint* mTrackCapsuleSphereRope;
	//std::vector<DistanceConstraint> mNotInPipelineJoints;

	void CreateLattice();
};