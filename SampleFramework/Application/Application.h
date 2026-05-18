#pragma once

#include "Display/ApplicationWindow.h"
#include "Renderer/Renderer.h"
#include "Camera.h"


#include "EditorImGui.h"

#include "Collision/Shapes/Shape.h"

#include "ApplicationUtil.h"

#include "Vortrix/Visuals/RenderSettings.h"


class PhysicsAppSetting
{
public:
	struct State
	{
		float frameTime = 0.0f;
		int subSteps = 0;
		float accumulator = 0.0f;
		int maxAttainedSubStep = 0;
		float maxAttainedTime = 0.0f;

		bool paused = false;
		int multipleStep = 0;//nSteps;

		int nStep = 5;
	};

	int Rate() const { return mFrequency; }
	float FixedTimeStep() const { return mFixedTimeStep; }

	const State& StateStats() const { return mState; }
	State& StateStats() { return mState; }

	void SetFrequency(int hz)
	{
		mFrequency = hz;
		mFixedTimeStep = 1.0f / static_cast<float>(hz);
	}

	bool AllowStep()
	{
		bool single_step = mState.multipleStep > 0; //(bool)(mState.nSteps);
		bool allow = !(mState.paused && !single_step);
		mState.paused |= single_step;
		mState.multipleStep -= (mState.multipleStep > 0) ? 1 : 0;
		return allow;
	}

	int mMaxSubStep = 3;
private:
	int mFrequency = 60.0f;
	float mFixedTimeStep = 1.0f / 60.0f;

	State mState;
};

struct DirectionDebugProperties
{
	vx::Vec3 worldRefOrigin = vx::Vec3(0, 1.0f, 0);

	vx::Float3 viewPosition{ 1.0f, 3.5f, 3.3f };
	float halfSize = 2.0f;
	vx::Float3 viewTarget{ 0.0f, 1.1f, 0.0f };
	float arrowWidth = 0.1f;

	float arrowHeight = 0.2f;
	float lightGizmosBaseOffset = 0.35f;

	bool enableDraw = true;

	bool dirty = false;
};



//forward declares
namespace InputSystem {
	class EventHandler;
}
namespace vx {
	namespace Particles {
		class ParticleWorld;
		class Particle;
	}
	class Body;
	class PhysicsWorld;
}
class Scenario;

class Application
{
private:
	ApplicationWindow mWindow;


	//Remove later
	Renderer mRenderer;
	class DebugGizmosRenderer* mDebugGizmos = nullptr;

	Camera mCamera;

	//Pointer to global static event handler
	InputSystem::EventHandler* mPtrInputEventHandle;


	//class VPHX::ParticleWorld* mParticleWorld = nullptr;
	vx::Particles::ParticleWorld* mParticleWorld = nullptr;
	vx::PhysicsWorld* mPhysicsWorld = nullptr;

	class ScenarioCatergory// : public vx::NonCopyable
	{
	public:
		const char* name;
		std::vector<ScenarioCatergory> catergories;
		std::vector<vx::Scope<Scenario>> scenarios;
	};
	ScenarioCatergory mScenarioCatergoies;
	Scenario* mCurrScenario = nullptr;

	float mRunningScenarioDuration = 5.0f;
	float mCurrentScenarioDuration = 0.0f;
	std::vector<Scenario*> mPendingRunScenarios;
	void RunAllScenarioWindow();
	void SaveCurrentScenarionWindow();

	vx::RenderSettings mPhysicsRenderSettings{};
	SpawnObjectCanon mTestCanon{};

	EditorImGui mUI;

public:
	Application() = delete;
	Application(const ApplicationSpecification& app_spec);

	virtual ~Application();

	//main loop
	void Run();
	void ResetWorld(bool& reset_flag);
	void ResetParticleWorld(bool& reset_flag);

private:

	void PhysicsStep(double frame_dt);

	bool bFailLaunch = false;

	struct PhysicsDebugState
	{
		bool keepSettingsOnReset = true;
		bool triggerReset = false;

		bool triggerParticleReset = false;

		vx::Vec3 spawnObjOrigin;
		vx::Quat spawnOrientation;
	}mPhysicsDebugState;
	//bool bIsPhysicsStepPaused = false;

	double mFrameDeltaTime = 0.0f;
	double mLastFrameTime = 0.0f;
	double mMaxAttainedFrameDeltaTime = 0.0f;
	float mTimeScale = 1.0f;

	void UpdateCamera(float dt);
	void ResetCamera();

	void CheckInputs();
	bool CheckInputsBlocked();


	void Quit();

private:
	//test objects 
	void CreateTestObjects();
	vx::Vec3 mOrbitOrigin = vx::Vec3(0.0f, 15.0f, 0.0f);
	vx::Vec3 mTwentyRndPos[70] = {};
	bool mRndTrueOrFalse[70] = {};

	vx::Mat44 mPlaneTransform = vx::Mat44(1.0f);
	vx::Mat44 mSphereTransform = vx::Mat44(1.0f);


	void OnRenderer();
	void OnApplicationDebugDraw();
	void SubmitRenderObjects();

	void OnExpandDrawScenarioCatergory(const ScenarioCatergory& catergory);
	void OnDrawImGuiOverlays();

	void OnDrawBodiesOverlays();

	void PhysicsSettingItemOverlays();
	void DrawProfileOverlay();
	void CreateNewPhysicsBodyWindow();
	void PhysicsCollisionWindows();
	void DrawUIRendererResourcesPanel(/*bool* p_open*/);

	void DrawUICameraStatePanel();
	void DrawUI_LightingPanel();
	void DrawUI_ShadowPanel();
	void PhysicsInteraction();

	void DrawHelpWindow(bool& p_open);

	void DrawBVHNodesOverlay(bool& open);

	ExternalEffectDynamicBodyInfo mExternalEffectDynamicBodyInfo;
	void DrawApplyExternalEffectDynamicBody(bool* p_open);

	AppCameraConfig mCameraConfig;

	//Test Util
	//float mMaxRenderTime = 0.0f;



private:

	PhysicsAppSetting mPhysicsAppSetting;

	CreatePhysicsObjectSettings mNewPhyObjectSettings{};
	vx::Particles::Particle* mLastAnchor = nullptr; 

	DebugAngularImpulse mDebugAngularImpulse;
	bool bShowDebugRotation = false;

	ShapeArena mShapeArena;

	std::string mAppName = "";
	DirectionDebugProperties mLightDebugProps;
};

Application* CreateApplication(const ApplicationSpecification& app_spec);