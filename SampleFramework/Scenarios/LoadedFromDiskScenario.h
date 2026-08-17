#include "Scenario.h"
#include "SampleFramework/ScenarioSerialiser.h"


class LoadedFromDiskScenario : public Scenario
{
public:

	void Init(vx::PhysicsWorld* i_world) override
	{
		Scenario::Init(i_world);

		Camera dummy;
		Camera& cam = (mAppCamera) ? *mAppCamera : dummy;
		serialiser::Deserialise(i_world, this, mName, mInfo, cam, mFilePath.Data());
	}
	const char* Name() override { return  mName.c_str(); }
	const char* Info() override
	{
		return mInfo.c_str();
	}

//private:
	vx::StackString<128> mFilePath;

	std::string mName;
	std::string mInfo;
};