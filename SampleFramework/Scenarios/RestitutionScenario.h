#pragma once
#include "Scenario.h"


class RestitutionScenario : public Scenario
{
public:
	void Init(vx::PhysicsWorld* i_world) override;
	const char* Name() override { return "Restitution Scenario"; }
	const char* Info() override 
	{
		return "Test bodies resitituion cofficent";
	}

	~RestitutionScenario();
//
//private:
//	struct CacheData
//	{
//		vx::ECombineMode restitutionCombine;
//		uint32_t velocityIterations = 10;
//	};
//	CacheData mCacheData;
};