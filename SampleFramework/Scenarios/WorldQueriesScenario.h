#pragma once
#include "Scenario.h"

#include "Vortrix/Collision/RayCast.h"

class WorldQueriesScenario : public Scenario
{
public:
	void Init(vx::PhysicsWorld* i_world) override;
	const char* Name() override { return "World Queries Scenario"; }
	const char* Info() override
	{
		return "Experiment with Raycast, etc";
	}

	void PostPhysicsStep(float dt) override;
	void OnUI() override;

private:
	vx::RayCast mExperimentRay;

	enum class EQueryType : vx::uint8
	{
		CastRayClosest,
		CastRayAny, 
		CastRayAll
	};

	EQueryType mQueryType = EQueryType::CastRayClosest;
	static constexpr const char* kQueryTypeNames = 
		"CastRayClosest\0"
		"CastRayAny\0"
		"CastRayAll\0"
		"\0";


	void CastRayClosest();
	void CastRayAny();
	void CastRayAll();
};