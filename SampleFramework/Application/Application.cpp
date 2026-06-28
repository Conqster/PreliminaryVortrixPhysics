#include "Core/HeapMemoryProfile.h"

#include "Application.h"
#include "Input/InputSystem.h"

#include <GLFW/glfw3.h>

#include "Utils/Util.h"

#include "Vortrix/Particles/ParticleWorld.h"
#include "Vortrix/Particles/Particle.h"
#include "Vortrix/PhysicsWorld.h"

#include "Vortrix/Core/Profiler.h"
#include "Renderer/DebugGizmosRenderer.h"
//#include "Vortrix/ForceSolver/Particle2PointOscillatingSpringSolver.h"



#include <external/imgui/imgui.h>
#include <external/imgui/imgui_impl_glfw.h>
#include <external/imgui/imgui_impl_opengl3.h>

#include "Dynamics/Body/Body.h"

#include "Collision/Shapes/Shape.h"
#include "Collision/Shapes/SphereShape.h"
#include "Collision/Shapes/BoxShape.h"
#include "Collision/Shapes/CapsuleShape.h"

#include "Collision/Broadphase/BroadphasePair.h"
#include "Collision/Broadphase/BVHBroadphase.h"
#include <algorithm>


#include "Vortrix/Maths/ViewProjection.h"

#include "Vortrix/Collision/Narrowphase/NarrowphaseQueryStat.h"
#include "Vortrix/Visuals/RenderSettings.h"


#include "Scenarios/SimpleBasicScenario.h"
#include "Scenarios/RestitutionScenario.h"
#include "Scenarios/FrictionScenario.h"
#include "Scenarios/BoxStackScenario.h"
#include "Scenarios/BoxPyramidStackScenario.h"
#include "Scenarios/JengaScenario.h"
#include "Scenarios/WorldQueriesScenario.h"
#include "Scenarios/JointScenario.h"
#include "Scenarios/PersistentContactScenario.h"
#include "Scenarios/RagdollScenario.h"

#include "Core/ScratchAllocator.h"

Application* CreateApplication(const ApplicationSpecification& app_spec)
{
	return new Application(app_spec);
}



using namespace InputSystem;
using namespace vx;




void Application::RunAllScenarioWindow()
{
	SaveCurrentScenarionWindow();
	if (ImGui::Begin("Run All Scenarios"))
	{
		ImGui::SliderFloat("Run scenario duration", &mRunningScenarioDuration, 0.1f, 10.0f);
		ImGui::Text("Current: %f", mCurrentScenarioDuration);
		static int triggered_count = 0;
		int ran = vx::VxMax(int(triggered_count-mPendingRunScenarios.size()), 0);
		ImGui::Text("Running: %d of %d", ran, triggered_count);
		ImGui::Text("Pending Count: %d", int(mPendingRunScenarios.size()));
		//bool run_all = false;
		if (ImGui::Button("Run All"))
		{
			//loop through scenarios
			std::vector<ScenarioCatergory*> _catergories;
			_catergories.push_back(&mScenarioCatergoies);
			while (!_catergories.empty())
			{
				ScenarioCatergory* c = _catergories.front();
				for (auto& _c : c->catergories)
					_catergories.push_back(&_c);

				for (auto& s : c->scenarios)
					mPendingRunScenarios.push_back(s.get());

				//swap to back then pop
				std::swap(_catergories.front(), _catergories.back());
				_catergories.pop_back();
			}

			mCurrentScenarioDuration = mRunningScenarioDuration + 0.1f;
			triggered_count = mPendingRunScenarios.size();
		}

	}
	ImGui::End();
}

void Application::SaveCurrentScenarionWindow()
{

	if (ImGui::Begin("Save Scenario"))
	{
		int _max_bodies, _max_body_pairs, _max_constact_constraints;
		PhysicsWorld::GenerateWorldDefaultConfig(_max_bodies, _max_body_pairs, _max_constact_constraints);

		int avg_max_body_pair = 8, avg_contact_per_body = 4;
		static bool use_def_phy_config = true;
		ImGui::Checkbox("Use Default Physics Config", &use_def_phy_config);
		ImGui::DragInt("Max Bodies", &_max_bodies);
		ImGui::DragInt("Avr Max Body Pair", &avg_max_body_pair);
		ImGui::DragInt("Avr Contact Per Body", &avg_contact_per_body);
		static char scenario_name[64] = "New Scenario v1";
		ImGui::InputText("Name", &scenario_name[0], sizeof(char) * 64);

		if (ImGui::Button("Print name"))
		{
			vx::StackString<64> _name;
			_name << scenario_name;
			VX_LOG_INFO("Name test: ", _name);
		}
	}
	ImGui::End();
}



Application::Application(const ApplicationSpecification& app_spec)
{
	mLastFrameTime = glfwGetTime();

	VX_LOG_INFO("Launching Application Program, \n\tName: ",
		app_spec.name, "\n\tWindow Size: {", 
		app_spec.windowSize[0], ", ", 
		app_spec.windowSize[1], "}\n\tDisableBindlessSupport: ", 
		app_spec.disableBindlessSupport ? "True" : "False");

	mAppName = app_spec.name;

	WindowSpecification wind_spec;
	wind_spec.windowSize[0] = app_spec.windowSize[0];
	wind_spec.windowSize[1] = app_spec.windowSize[1];
	wind_spec.windowPos[0] = app_spec.windowPos[0];
	wind_spec.windowPos[1] = app_spec.windowPos[1];
	wind_spec.disableBindlessSupport = app_spec.disableBindlessSupport;


	ImageData img_data;
	if (!wind_spec.iconPixel)
	{
		TextureFormat format = TextureFormat::RGBA8;
		img_data = TextureLoader::LoadFromFile("assets/textures/icon48p.png", false, format);
		wind_spec.iconWidth = vx::VxClamp((int)img_data.width, 16, 48);
		wind_spec.iconHeight = vx::VxClamp((int)img_data.height, 16, 48);
		wind_spec.iconPixel = (unsigned char*)img_data.pixels;
	}

	if (!mWindow.Init(app_spec.name.c_str(), wind_spec, app_spec.launchFullScreen, " -" VX_BUILD_STR))
		bFailLaunch = true;

	TextureLoader::Free(img_data);


	if (!bFailLaunch)
	{
		mRenderer.Initialise(&mWindow);

		mPtrInputEventHandle = &InputSystem::EventHandler::Instance();
		mPtrInputEventHandle->CreateCallbacks(mWindow.GetWindow());


		//mCamera = Camera(vx::Vec3(0.0f, 5.0f, 14.00f), 0.0f, 180.0f, 17.5f, 4.0f);
		mCamera = Camera(vx::Vec3(0.0f, 5.0f, 14.00f), 0.0f, 180.0f, 40.0f, 10.0f);

		
		mRenderer.SetDirectionalLightColour(vx::Colour(vx::Vec3(0.8f)));
		mRenderer.SetDirectionalLightIntensity(0.7f);
		//mRenderer.SetDirectionalLightDir(vx::Vec3(-0.074, -0.519f, 0.852f));
		mRenderer.SetDirectionalLightDir(vx::Vec3(0.398f, -0.581f, -0.710f));

		mDebugGizmos = new DebugGizmosRenderer();
		if (!mDebugGizmos->Init(&mWindow))
		{
			delete mDebugGizmos;
			mDebugGizmos = nullptr;
			VX_LOG_WARN("[APP -- {", app_spec.name.c_str(), "}], Failed to initialise debug gizmos renderer");
		}
		//mDebugGizmos->SetLineWidth(4.0f);
		mDebugGizmos->SetLineWidth(2.0f);


		mUI.Initialise(mWindow.GetWindow());
	}


	/// generate test scenario
	/// later do not allocate mem not yet 
	/// has they could be tens of scenario 
	/// and only one tenth of might be run in session
	mScenarioCatergoies.name = "Scenarios";
	mScenarioCatergoies.scenarios.push_back(vx::MakeScope<SimpleBasicScenario>());
	mScenarioCatergoies.scenarios.push_back(vx::MakeScope<WorldQueriesScenario>());
	mScenarioCatergoies.scenarios.push_back(vx::MakeScope<JointScenario>());
	mScenarioCatergoies.scenarios.push_back(vx::MakeScope<PersistentContactScenario>());
	mScenarioCatergoies.scenarios.push_back(vx::MakeScope<RagdollScenario>());
	
	ScenarioCatergory solver_scenarios;
	solver_scenarios.name = "Solvers";
	solver_scenarios.scenarios.push_back(vx::MakeScope<RestitutionScenario>());
	solver_scenarios.scenarios.push_back(vx::MakeScope<FrictionScenario22_5>());
	solver_scenarios.scenarios.push_back(vx::MakeScope<FrictionScenario45>());
	mScenarioCatergoies.catergories.push_back(std::move(solver_scenarios));

	ScenarioCatergory stacking_scenarios;
	stacking_scenarios.name = "Stacking";
	stacking_scenarios.scenarios.push_back(vx::MakeScope<BoxStackScenario>());
	stacking_scenarios.scenarios.push_back(vx::MakeScope<BoxPyramidStackScenario>());
	stacking_scenarios.scenarios.push_back(vx::MakeScope<JengaScenario>());
	mScenarioCatergoies.catergories.push_back(std::move(stacking_scenarios));


	mCurrScenario = mScenarioCatergoies.scenarios.at(4).get();
}

Application::~Application()
{
	//fix ?????, ui crash if programm closes early 
	//
	mUI.Shutdown();
	mRenderer.Destroy();

	delete mDebugGizmos;
	delete mPhysicsWorld;
	delete mParticleWorld;
	mDebugGizmos = nullptr;
	mPhysicsWorld = nullptr;
	VX_LOG_DEBUG("Closing application program...");
	mWindow.Destroy();
	mPtrInputEventHandle = nullptr;
}

void Application::Run()
{
	if (bFailLaunch)
		return;


	//SET_VPHX_PROFILER_GLOBAL_SAMPLE_RATE(60);
	SET_VX_PROFILER_GLOBAL_SAMPLE_RATE(1);
	SET_VX_PROFILER_SAMPLE_INTERVAL_SECONDS(0);
	SET_VX_PROFILER_FILTER(VX_PROFILER_DETERMINISTIC | VX_PROFILER_VARIABLERATE);
	SET_VX_PROFILER_ALLOW_CONSOLE_LOG(false);

	//main loop {delta time >> camera 
	while (mWindow.ProgramActive())
	{
		VX_MARK_NEW_FRAME;

		//VX_INFO("================== NEW FRAME ================== ");
		static float loop_time = 0.0f;

		double curr_frame_time = glfwGetTime();
		mFrameDeltaTime = curr_frame_time - mLastFrameTime;
		mLastFrameTime = curr_frame_time;

		/// Event handling
		{
			VX_VARIABLE_PROFILE_SCOPE("Poll and Handling Events")
			if (mPtrInputEventHandle)
				mPtrInputEventHandle->FlushFrameInputs();
			mWindow.PollEvents();
		}
		CheckInputs();

		if (mWindow.Minimised())
			continue;
		
		/// Camera Update
		UpdateCamera(mFrameDeltaTime);

		////scale simulation time
		mFrameDeltaTime *= mTimeScale;
		mRenderer.mTime = mFrameDeltaTime;

		/// Physics simulation Update
		if (mPhysicsAppSetting.AllowStep())
		{
			if (!CheckInputsBlocked())
				PhysicsInteraction();


			//hack to ensure 
			//physiucs has debug draw command to use
			mDebugGizmos->PushDrawCommand(mCamera.ProjMat(mWindow.GetAspectRatio()), mCamera.ViewMat(), nullptr);
			PhysicsStep(mFrameDeltaTime);
			mDebugGizmos->EndCurrentDrawCommand();
		}


		//quick 
		if (!mPendingRunScenarios.empty())
		{
			if (mCurrentScenarioDuration > mRunningScenarioDuration)
			{
				(mCurrScenario) ? mCurrScenario->OnClose() : void(0);
				mCurrScenario = mPendingRunScenarios.front();
				mPhysicsDebugState.triggerReset = true;
				mCurrentScenarioDuration = 0.0f;
				////swap to back then pop
				//if (mPendingRunScenarios.size() == 1)
				//{
				//	//mFrameDeltaTime;
				//	mCurrentScenarioDuration += 0.001f;
				//}
				std::swap(mPendingRunScenarios.front(), mPendingRunScenarios.back());
				mPendingRunScenarios.pop_back();
			}
			else
				mCurrentScenarioDuration += mFrameDeltaTime;
		}


		/// Physics Reset
		if (mPhysicsDebugState.triggerReset)
			ResetWorld(mPhysicsDebugState.triggerReset);
		if (mPhysicsDebugState.triggerParticleReset)
			ResetParticleWorld(mPhysicsDebugState.triggerParticleReset);




		/// Debug frame times
		if (mMaxAttainedFrameDeltaTime < mFrameDeltaTime)
		{
			mMaxAttainedFrameDeltaTime = mFrameDeltaTime;
			VX_LOG_DEBUG("Sim Time:", mMaxAttainedFrameDeltaTime, "ms");
		}

		OnRenderer();
		OnDrawImGuiOverlays();
		if (mCurrScenario)
		{
			//debg current scene 
			VX_VARIABLE_PROFILE_SCOPE("App Scenario UI Overlay");
			if (ImGui::Begin("Debug Scenario"))
			{
				ImGui::Text("Scene: %s", mCurrScenario->Name());
				ImGui::Text("Info: %s", mCurrScenario->Info());
			}
			ImGui::End();
			mCurrScenario->OnUI();
		}

		//========== RESOLVE FRAME ==========/
		{
			VX_VARIABLE_PROFILE_SCOPE("Resolve Application Frame");
			mUI.RenderFrame();
			mWindow.SwapBuffer();
		}
	}

}

void Application::ResetWorld(bool& reset_flag)
{
	VX_PROFILE_FUNCTION();
	vx::PhysicsWorldSettings phy_wld_setting = (mPhysicsWorld && mPhysicsDebugState.keepSettingsOnReset) ?
					mPhysicsWorld->GetSettings() : vx::PhysicsWorldSettings();

	delete mPhysicsWorld;

	/// note this resets camera, 
	/// but could be overwritten in Scenario if needed
	ResetCamera();

	mPhysicsWorld = new PhysicsWorld(phy_wld_setting);

	int _max_bodies, _max_body_pairs, _max_constact_constraints;
	PhysicsWorld::GenerateWorldDefaultConfig(_max_bodies, _max_body_pairs, _max_constact_constraints);
	mPhysicsWorld->Init(_max_bodies, _max_body_pairs, _max_constact_constraints);

	//PhysicsWorld::CreateSimpleWorld(mPhysicsWorld);
	if (mCurrScenario != nullptr)
	{
		mCurrScenario->OnClose();
		mCurrScenario->SetCamera(&mCamera);
		mCurrScenario->SetDebugGizmos(mDebugGizmos);
		mCurrScenario->SetWindow(&mWindow);
		mCurrScenario->SetAppEditor(&mUI);
		mCurrScenario->Init(mPhysicsWorld);

		vx::StackString<120> new_title(mWindow.GetBaseTitle());
		new_title << " {" << mCurrScenario->Name();
		if (new_title.Length() >= 120)
			VX_ASSERT(false, "want to add option to replace end with }");
		else
			new_title << "} -" VX_BUILD_STR;
		mWindow.ChangeWindowTitle(new_title.Data());
	}
	else
		PhysicsWorld::CreateSimpleWorld(mPhysicsWorld);



	reset_flag = false;
}

void Application::ResetParticleWorld(bool& reset_flag)
{
	VX_PROFILE_FUNCTION();
	delete mParticleWorld;
	mParticleWorld = new Particles::ParticleWorld(Particles::kParticleLimit, Vec3(0.0f, -9.85f, 0.0f));
	Particles::ParticleWorld::CreateSimpleSampleWorld(mParticleWorld);
	reset_flag = false;
}

void Application::PhysicsStep(double frame_dt)
{
	VX_PROFILE_FUNCTION();

	auto physics_dt = mPhysicsAppSetting.FixedTimeStep();

	PhysicsAppSetting::State& curr_state = mPhysicsAppSetting.StateStats();
	curr_state.accumulator += mFrameDeltaTime;
	curr_state.subSteps = 0;

	bool physics_scenario_world = mPhysicsWorld && mCurrScenario;
	if (physics_scenario_world)
	{
		VX_PROFILE_SCOPE("Pre Physics Step");
		mCurrScenario->PrePhysicsStep(physics_dt);
	}

	{
		VX_PROFILE_SCOPE("Physics-Update", &curr_state.frameTime, false);
		while (curr_state.accumulator >= physics_dt && curr_state.subSteps < mPhysicsAppSetting.mMaxSubStep)
		{

			if (mPhysicsWorld)
				mPhysicsWorld->StepSimulation(physics_dt);
			if (mParticleWorld)
				mParticleWorld->StepSimulation(physics_dt);

			curr_state.accumulator -= physics_dt;
			curr_state.subSteps++;
		}
	}
	if (physics_scenario_world)
	{
		VX_PROFILE_SCOPE("Post Physics Step");
		mCurrScenario->PostPhysicsStep(physics_dt);
	}



	if (curr_state.frameTime > curr_state.maxAttainedTime)
	{
		VX_LOG_DEBUG("Max Physics time: ", curr_state.frameTime, "ms");
		if (curr_state.frameTime < 30.0f)
			curr_state.maxAttainedTime = curr_state.frameTime;
	}

	curr_state.maxAttainedSubStep = vx::VxMax(curr_state.maxAttainedSubStep, curr_state.subSteps);
}

void Application::UpdateCamera(float dt)
{
	if (CheckInputsBlocked())
		return;

	if (mWindow.GetLockCursor())
		mCamera.Rotate(Input::GetMouseAxisFloat(IAxis::Vertical), Input::GetMouseAxisFloat(IAxis::Horizontal), dt);


	//inputs
	if (Input::GetKey(IKeyCode::A))
		mCamera.Translate(-mCamera.GetRight(), float(dt));
	if (Input::GetKey(IKeyCode::D))
		mCamera.Translate(mCamera.GetRight(), float(dt));
	if (Input::GetKey(IKeyCode::W))
		mCamera.Translate(mCamera.GetForward(), float(dt));
	if (Input::GetKey(IKeyCode::S))
		mCamera.Translate(-mCamera.GetForward(), float(dt));
	if (Input::GetKey(IKeyCode::Q))
		mCamera.Translate(-mCamera.GetUp(), float(dt));
	if (Input::GetKey(IKeyCode::E))
		mCamera.Translate(mCamera.GetUp(), float(dt));


	//auto p = mCamera.GetPosition();
	//printf("Cam Pos; x: %.2ff, y: %.2ff, z: %.2ff\n", p.x, p.y, p.z);
}

void Application::ResetCamera()
{
	Camera::State cam_state(vx::Vec3(0.0f, 5.0f, 14.00f), 0.0f, 180.0f);
	Camera::Properties cam_props(40.0f, 10.0f);

	mCamera.SetState(cam_state);
	mCamera.SetProperties(cam_props);
}

void Application::CheckInputs()
{
	VX_VARIABLE_PROFILE_FUNCTION();
	if (Input::GetKeyDown(IKeyCode::R))
	{
		(mCurrScenario) ? mCurrScenario->OnClose() : void(0);
		mPhysicsDebugState.triggerReset = true;
	}
	
	bool ctr_pressed = Input::GetKey(IKeyCode::RightControl) || Input::GetKey(IKeyCode::LeftControl);
	bool alt_pressed = Input::GetKey(IKeyCode::RightAlt) || Input::GetKey(IKeyCode::LeftAlt);
	bool shift_pressed = Input::GetKey(IKeyCode::RightShift) || Input::GetKey(IKeyCode::LeftShift);

	if (ctr_pressed && Input::GetKeyDown(IKeyCode::N))
		mNewPhyObjectSettings.openWindow = !shift_pressed;

	auto& physics_state = mPhysicsAppSetting.StateStats();
	if (Input::GetKeyUp(IKeyCode::P) || Input::GetKeyUp(IKeyCode::Pause))
		physics_state.paused = !physics_state.paused;
		//mPhysicsDebugState.isStepPaused = !mPhysicsDebugState.isStepPaused;

	if (Input::GetKeyUp(IKeyCode::F10))
		physics_state.multipleStep = (Input::GetKey(IKeyCode::LeftShift)) ? physics_state.nStep : 1;

	if (Input::GetKeyUp(IKeyCode::V))
		mWindow.SetVSync(!mWindow.GetVSync());

	if (Input::GetMousePressed(IKeyCode::MouseRightButton))
		mWindow.ToggleLockCursor();

	if(!CheckInputsBlocked())
	{


		//directional light location 
		float light_move_speed = 2.5f;
		auto light_pos = mRenderer.GetDirectionalLightDir();
		if (Input::GetKey(IKeyCode::J))
			light_pos += vx::Vec3(1.0f, 0.0f, 0.0f) * float(mFrameDeltaTime) * light_move_speed;
		if (Input::GetKey(IKeyCode::L))
			light_pos += vx::Vec3(-1.0f, 0.0f, 0.0f) * float(mFrameDeltaTime) * light_move_speed;

		if (Input::GetKey(IKeyCode::K))
			light_pos += vx::Vec3(0.0f, 0.0f, -1.0f) * float(mFrameDeltaTime) * light_move_speed;
		if (Input::GetKey(IKeyCode::I))
			light_pos += vx::Vec3(0.0f, 0.0f, 1.0f) * float(mFrameDeltaTime) * light_move_speed;


		if (Input::GetKey(IKeyCode::U))
			light_pos += vx::Vec3(0.0f, 1.0f, 0.0f) * float(mFrameDeltaTime) * light_move_speed;
		if (Input::GetKey(IKeyCode::O))
			light_pos += vx::Vec3(0.0f, -1.0f, 0.0f) * float(mFrameDeltaTime) * light_move_speed;

		mRenderer.SetDirectionalLightDir(light_pos.Normalised());

	}

}

bool Application::CheckInputsBlocked()
{
	//when mouse cursoer is not locked & ui want blocking
	return !mWindow.GetLockCursor() && mUI.UIBlockingInput(); 
}

void Application::Quit()
{
	mWindow.Close();
}



void Application::OnRenderer()
{
	VX_VARIABLE_PROFILE_FUNCTION();
	//static float render_time = 0.0f;

	//VPHX_VARIABLE_PROFILE_SCOPE("Render-Begin", &render_time, false);
	{
		VX_VARIABLE_PROFILE_SCOPE("Application Pre Render");
		mRenderer.BeginFrame(&mCamera, vx::Colour(0.0f, 0.2f, 0.5f, 1.0f));

		SubmitRenderObjects();
		if (mPhysicsWorld)
			mPhysicsWorld->OnDrawBodies(&mRenderer, mPhysicsRenderSettings);
	}

	{
		VX_VARIABLE_PROFILE_SCOPE("Application Render Draw");
		mRenderer.ShadowPass();
		mRenderer.DrawPass();
	}


	{
		VX_VARIABLE_PROFILE_SCOPE("Application Pre Debug Gizmos");
			mDebugGizmos->PushDrawCommand(mCamera.ProjMat(mWindow.GetAspectRatio()), mCamera.ViewMat(), nullptr);

		if (mPhysicsWorld)
			mPhysicsWorld->OnDebugDraw(mDebugGizmos);

		if (mPhysicsRenderSettings.drawWorldAxes)
		{
			vx::Vec3 rt = vx::Vec3::Right() * mPhysicsRenderSettings.worldAxesLength;
			vx::Vec3 up = vx::Vec3::Up() * mPhysicsRenderSettings.worldAxesLength;
			vx::Vec3 fwd = vx::Vec3::Forward() * mPhysicsRenderSettings.worldAxesLength;
			vx::Vec3 c = vx::Vec3::Zero();
			mDebugGizmos->DrawLine(c, c + rt, vx::Colour::sRed);
			mDebugGizmos->DrawLine(c, c + up, vx::Colour::sGreen);
			mDebugGizmos->DrawLine(c, c + fwd, vx::Colour::sBlue);
		}

		if (mParticleWorld)
		{
			mParticleWorld->OnDebug(mDebugGizmos);
			for (const auto& p : mParticleWorld->GetParticles())
				mDebugGizmos->DrawLine(p.mPosition, p.mPosition + p.GetVelocity().Normalise(), vx::Colour(vx::Vec3::Forward()));

		}
		OnApplicationDebugDraw();
	}


	{
		VX_VARIABLE_PROFILE_SCOPE("Application DebugGizmos Draw");
		mDebugGizmos->Flush(mRenderer.GetTestRTPtr());

		//FrameBlitInfo blit_info;
		//blit_info.readFBO = &mRenderer.GetTestRTPtr()->GetFramebuffer();
		//blit_info.srcSize = { 125, 125 };
		////blit_info.dstSize = { (float)mWindow.GetWidth(), (float)mWindow.GetHeight() };
		//blit_info.dstSize = { 125, 125 };
		//blit_info.mask = GL_DEPTH_BUFFER_BIT;
		//Framebuffer::Blit(blit_info);
		//glBindFramebuffer(GL_FRAMEBUFFER, 0);
		////remove

		GLCall(glViewport(0, 0, mWindow.GetWidth(), mWindow.GetHeight()));

		//glDisable(GL_BLEND);
		GLCall(glEnable(GL_DEPTH_TEST));

		mDebugGizmos->ExecuteDraws();
	}

	mRenderer.EndFrame();
}

void Application::OnApplicationDebugDraw()
{
	VX_VARIABLE_PROFILE_FUNCTION();
	if (!mDebugGizmos)return;


	/////////////////////////////////////////////////
	/////////////////////DRAW IMPLUSE APP////////////
	/////////////////////////////////////////////////
#pragma region DRAW IMPLUSE APPILCAION ARROW
	if(mDebugAngularImpulse.draw)
	{
		vx::Colour col(1.0f, 0.0f, 0.0f);
		vx::Vec3 apply_impluse_world_pt = vx::Vec3::LoadFloat3Raw(mDebugAngularImpulse.pointA);

		/// apply_impluse_world_pt (x)
		/// drawing 
		/// 
		/// x <-- ; arrow point to the point x
		/// i.e arrow cone end = x 
		/// start = x - impluse displacement
		/// 
		vx::Vec3 J = vx::Vec3::LoadFloat3Raw(mDebugAngularImpulse.impluse);
		mDebugGizmos->DrawArrowCone(apply_impluse_world_pt - J, apply_impluse_world_pt, 0.2f, 0.5f, 0.2f);
	}
#pragma endregion



	/////////////////////////////////////////////////
	/////////////////////DRAW CROSS HAIR APP////////////
	/////////////////////////////////////////////////
#pragma region DRAW IMPLUSE APPILCAION ARROW
	{
		vx::Colour col = vx::Colour::sRed;
		float half_size = 0.0625f;
		vx::Vec3 c = mCamera.GetPosition() + mCamera.GetForward();
		vx::Vec3 _long = mCamera.GetUp() * half_size;
		mDebugGizmos->DrawLine(c - _long, c + _long, col);
		vx::Vec3 lat = mCamera.GetRight() * half_size;
		mDebugGizmos->DrawLine(c - lat, c + lat, col);
	}
#pragma endregion


#pragma region DRAW CREATE VIEW BASIS
	{
		///lets create a camera that 
		/// face foward 
		vx::Mat44 proj = vx::Perspective(vx::DegToRad(60.0f), 1.0f, 0.1f, 70.0f);
		vx::Mat44 view = vx::ViewMatrixFromBasis(vx::Vec3(0.0f, 0.0f, -2.2f), vx::Vec3::Right(), vx::Vec3::Up(), vx::Vec3::Forward());
		mDebugGizmos->PushDrawCommand(proj, view, mRenderer.GetTestRTPtr());

		float size = 0.5f;

		vx::Vec3 debug_origin = vx::Vec3::Zero();
		vx::Vec3 fwd = mCamera.GetForward();
		vx::Vec3 u = mCamera.GetUp();
		vx::Vec3 rt = mCamera.GetRight();

		vx::Mat44 view_mat = vx::ViewMatrixFromBasis(debug_origin, rt, u, -fwd);
		mDebugGizmos->DrawBasis(view_mat, 0.1f, 0.2f);
		mDebugGizmos->EndCurrentDrawCommand();
	}
#pragma endregion



	/// DRAW LIGHT DIR AXES
	/// wc DRAW WORLD ALIGN XY DISC
	/// wc DRAW WORLD ALIGN ZY & XZ DISC
	/// wc DRAW WORLD AXES
	/// DRAW LIGHT DIRECTION WITH ARROW AND DISC
	/// DRAW LIGHT DIR AXES
#pragma region DRAW DIRECTIONAL LIGHT DETAILS

	if(mLightDebugProps.enableDraw)
	{
		IRenderTarget* dir_light_debug_rt = mRenderer.GetmDirLightDebugRTPtr();
		dir_light_debug_rt->Bind();
		Colour col = vx::Colour(0.25f, 0.25f, 0.25f);
		glClearColor(col.R(), col.G(), col.B(), col.A());
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		GLCall(glBindFramebuffer(GL_FRAMEBUFFER, 0));
		GLCall(glViewport(0, 0, mWindow.GetWidth(), mWindow.GetHeight()));

		/// MOVE OUT
		/// MOVE OUT
		/// MOVE OUT
		/// MOVE OUT
		static vx::Mat44 proj = vx::Perspective(vx::DegToRad(70.0f), 1.0f, 0.1f, 10.0f);
		static vx::Mat44 view = vx::LookAt(
			vx::Vec3::LoadFloat3Raw(mLightDebugProps.viewPosition),
			vx::Vec3::LoadFloat3Raw(mLightDebugProps.viewTarget), vx::Vec3::Up());

		if (mLightDebugProps.dirty)
		{
			proj = vx::Perspective(vx::DegToRad(70.0f), 1.0f, 0.1f, 10.0f);
			view = vx::LookAt(
				vx::Vec3::LoadFloat3Raw(mLightDebugProps.viewPosition),
				vx::Vec3::LoadFloat3Raw(mLightDebugProps.viewTarget), vx::Vec3::Up());
		}

		//view = vx::LookAt(vx::Vec3(1.0f, 3.4f, -2.6f), vx::Vec3(0.0f, 1.1f, 0.0f), vx::Vec3::Up());

		mDebugGizmos->PushDrawCommand(proj, view, dir_light_debug_rt);
		//mDebugGizmos->PushDrawCommand(mCamera.ProjMat(mWindow.GetAspectRatio()), mCamera.ViewMat(), nullptr);


		vx::Vec3 light_dir = mRenderer.GetDirectionalLightDir().Normalised();

		/// LIGHT BASIS
		vx::Vec3 forward = -light_dir;
		//vx::Vec3 right = forward.NormalisedPerpendicular();
		vx::Vec3 world_up = VxAbs(light_dir.Y()) < 0.99f ? vx::Vec3::Up() : vx::Vec3::Right();
		vx::Vec3 right = world_up.Cross(forward).Normalised();
		vx::Vec3 up = right.Cross(forward).Normalised();


		/// start dynamic draws
		/// later write alogrithm to bake line segments 
		/// into a mesh, to draw per frame which only updates when like direction changes

		/// DRAW WORLD ALIGN XY DISC
		for (float r = mLightDebugProps.halfSize; r > 0; r -= (mLightDebugProps.halfSize / 4))
		{
			//for now, easy modification 
			vx::Vec3 wr = vx::Vec3::Right();
			vx::Vec3 wf = vx::Vec3::Forward();

			mDebugGizmos->DrawWireDisc(mLightDebugProps.worldRefOrigin, r, vx::Colour(vx::Vec3::Up()), wr, wf);
		}
		/// DRAW WORLD ALIGN ZY & XZ DISC
		mDebugGizmos->DrawHalfWireDisc(mLightDebugProps.worldRefOrigin, mLightDebugProps.halfSize, vx::Colour(vx::Vec3::Right()), vx::Vec3::Forward(), vx::Vec3::Up());
		mDebugGizmos->DrawHalfWireDisc(mLightDebugProps.worldRefOrigin, mLightDebugProps.halfSize, vx::Colour(vx::Vec3::Forward()), vx::Vec3::Right(), vx::Vec3::Up());

		/// DRAW WORLD AXES
		mDebugGizmos->DrawArrow(mLightDebugProps.worldRefOrigin, mLightDebugProps.worldRefOrigin + vx::Vec3::Forward() * mLightDebugProps.halfSize, vx::Vec3::Right(), mLightDebugProps.arrowWidth, mLightDebugProps.arrowHeight, vx::Colour(vx::Vec3::Forward()));
		mDebugGizmos->DrawArrow(mLightDebugProps.worldRefOrigin, mLightDebugProps.worldRefOrigin + vx::Vec3::Up() * mLightDebugProps.halfSize, vx::Vec3::Forward(), mLightDebugProps.arrowWidth, mLightDebugProps.arrowHeight, vx::Colour(vx::Vec3::Up()));
		mDebugGizmos->DrawArrow(mLightDebugProps.worldRefOrigin, mLightDebugProps.worldRefOrigin + vx::Vec3::Right() * mLightDebugProps.halfSize, vx::Vec3::Up(), mLightDebugProps.arrowWidth, mLightDebugProps.arrowHeight, vx::Colour(vx::Vec3::Right()));


		///DRAW LIGHT DIRECTION WITH ARROW AND DISC
		vx::Vec3 light_gizmo_origin(mLightDebugProps.worldRefOrigin + (-light_dir * mLightDebugProps.lightGizmosBaseOffset));
		float light_half_size = mLightDebugProps.halfSize - 0.35f;
		vx::Vec3 abituary = vx::VxAbs(light_dir.Y()) < 0.99f ? vx::Vec3::Up() :
			vx::VxAbs(light_dir.X()) < 0.99f ? vx::Vec3::Right() :
			vx::Vec3::Forward();
		//vx::Vec3 light_dir_2_view = abituary.Cross(light_dir);
		vx::Vec3 light_dir_2_view = mCamera.GetRight();
		if (VxAbs(light_dir_2_view.Dot(light_dir)) > 0.99f)
			light_dir_2_view = mCamera.GetUp();
		light_dir_2_view = light_dir_2_view.Reject(light_dir);
		light_dir_2_view.Normalise();
		mDebugGizmos->DrawArrow(light_gizmo_origin - (light_dir * light_half_size), light_gizmo_origin, light_dir_2_view, mLightDebugProps.arrowWidth, mLightDebugProps.arrowHeight, vx::Colour(0.9f, 0.7f, 0.0f));
		//orientation debug disc
		mDebugGizmos->DrawWireDisc(mLightDebugProps.worldRefOrigin, mLightDebugProps.lightGizmosBaseOffset, vx::Colour(vx::Vec3(0.9f, 0.7f, 0.0f)), right, forward);

		//// DRAW LIGHT DIR AXES
		//project order
		// right
		// forward
		// up 
		// 
		// maybe 
		// right
		// up
		// forward
		vx::Vec3 world_basis[3] = {
			vx::Vec3::Right(),
			vx::Vec3::Forward(),
			vx::Vec3::Up(),
		};

		vx::Vec3 last_origin = mLightDebugProps.worldRefOrigin;
		float proj_length = 1.5f;
		float test_xy = proj_length;
		for (uint32_t i = 0; i < 3; ++i)
		{
			vx::Vec3 proj_vec = -light_dir.Project(world_basis[i]) * proj_length;
			vx::Vec3 next_origin = last_origin + proj_vec;
			mDebugGizmos->DrawLine(last_origin, next_origin, vx::Colour(world_basis[i] * 0.6f));
			last_origin = next_origin;
		}


		mDebugGizmos->EndCurrentDrawCommand();
	}
#pragma endregion


	/// RESET BACK TO DEFAULT
	mDebugGizmos->PushDrawCommand(mCamera.ProjMat(mWindow.GetAspectRatio()), mCamera.ViewMat(), nullptr);
}

void Application::SubmitRenderObjects()
{
	VX_VARIABLE_PROFILE_FUNCTION();
	//sphere :-> mesh & tranform 
	//sphere :-> mesh & tranform 
	//plane :-> mesh & tranform 

	//ground

	if(!mPhysicsWorld || !mPhysicsWorld->GetSettings().drawSettings.drawBodiesAsSolid)
	{
		vx::Mat44 trans = vx::Quat::FromAxisAngle(-vx::Vec3::Right(), vx::DegToRad(90.0f)).GetRotationMat44().ScaledLocal(vx::Vec3(100.0f));
		mRenderer.SubmitQuadPrimitive({ nullptr, trans,
			true, false, vx::Colour(0.7f, 0.7f, 0.7f, 1.0f), false }, vx::ERenderInstanceFlags::CastShadow | vx::ERenderInstanceFlags::ReceiveShadow);
	}

	//as particle
	if (mParticleWorld)
	{
		auto particles = mParticleWorld->GetParticles();

		for (const auto& p : particles)
		{

			mRenderer.SubmitSpherePrimitive(
				{
					nullptr,
					vx::Mat44::Translation(p.mPosition).ScaledLocal(vx::Vec3(p.GetRadius() * 2.0f)),
					//glm::translate(glm::mat4(1.0f), p.mPosition.AsGLM()) *
					//glm::scale(glm::mat4(1.0f), glm::vec3(p.GetRadius() * 2.0f)),
					true,
					true,
					vx::Colour(0.0f, 1.0f, 0.0f, 1.0f),
					true
				}, vx::ERenderInstanceFlags::CastShadow | vx::ERenderInstanceFlags::ReceiveShadow);
		}
	}

	//testing 
	if (bShowDebugRotation)
	{
		static vx::Mat44 test_tranform = Mat44::Translation(Vec3(5.0f, 5.0f, 0.0f));
		test_tranform = test_tranform.Multiply(Mat44::RotationX(DegToRad(15.0f) * mFrameDeltaTime) *
			Mat44::RotationY(DegToRad(30.0f) * mFrameDeltaTime) *
			Mat44::RotationZ(DegToRad(60.0f) * mFrameDeltaTime));
		mRenderer.SubmitCubePrimitive(
			{
				nullptr,
				test_tranform,
				true, true,
				vx::Colour(0.0f, 1.0f, 0.0f, 1.0f), true
			}, vx::ERenderInstanceFlags::CastShadow | vx::ERenderInstanceFlags::ReceiveShadow);
	}
	//draw canon
	mRenderer.SubmitCubePrimitive(
		{
			nullptr,
			vx::Mat44::Translation(mTestCanon.spawn).ScaledLocal(vx::Vec3(0.1f)),
			//glm::scale(glm::mat4(1.0f), glm::vec3(0.1f)),
			true, true,
			vx::Colour(1.0f, 0.0f, 0.0f, 1.0f), true
		}, vx::ERenderInstanceFlags::CastShadow | vx::ERenderInstanceFlags::ReceiveShadow);
	mRenderer.SubmitCubePrimitive(
		{
			nullptr,
			vx::Mat44::Translation(mTestCanon.back).ScaledLocal(vx::Vec3(0.1f)),
			//glm::translate(glm::mat4(1.0f), mTestCanon.back) *
			//glm::scale(glm::mat4(1.0f), glm::vec3(0.1f)),
			true, true,
			vx::Colour(1.0f, 0.0f, 0.0f, 1.0f), true
			
		}, vx::ERenderInstanceFlags::CastShadow | vx::ERenderInstanceFlags::ReceiveShadow);
}


void Application::OnExpandDrawScenarioCatergory(const ScenarioCatergory& catergory)
{

	if (ImGui::BeginMenu(catergory.name))
	{
		for (const auto& c : catergory.catergories)
			OnExpandDrawScenarioCatergory(c);

		for (const auto& scenario : catergory.scenarios)
					if (ImGui::MenuItem(scenario->Name()))
					{
						(mCurrScenario) ? mCurrScenario->OnClose() : void(0);
						mCurrScenario = scenario.get();
						mPhysicsDebugState.triggerReset = true;
						mPhysicsAppSetting.StateStats().paused = true;
					}

		ImGui::EndMenu();
	}

}





void Application::OnDrawImGuiOverlays()
{
	VX_VARIABLE_PROFILE_FUNCTION();
	mUI.BeginNewFrame();

	
	static bool open_phy_debug = true;
	static bool open_phy_entt_win = false;
	//static bool open_create_new_body_win = false;
	static bool open_app_win = true;
	static bool open_imgui_demo = false;
	static bool open_physics_collision_menu = false;
	static bool show_broadphase_insight = false;
	static bool open_phy_bvh_tree_debug = false;
	static bool open_help_window = false;
	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu("Menu"))
		{
			if (ImGui::MenuItem("V-Sync", "Ctrl+V"))
				mWindow.SetVSync(!mWindow.GetVSync());
			auto& physics_state = mPhysicsAppSetting.StateStats();
			if (ImGui::MenuItem((physics_state.paused) ? "Play Simulation" : "Pause Simulation", "PAUSE"))
				physics_state.paused = !physics_state.paused;
			if (ImGui::MenuItem("Quit", "Alt+F4"))
				Quit();
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("Edit"))
		{
			if (ImGui::MenuItem("Create New body", "SPACE BAR to spawn")) mNewPhyObjectSettings.openWindow = !mNewPhyObjectSettings.openWindow;
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("Windows"))
		{
			if (ImGui::MenuItem("Physics Config & Debug ")) open_phy_debug = !open_phy_debug;
			if (ImGui::BeginMenu("Collisions"))
			{
				if (ImGui::MenuItem("BroadPhase Insights")) show_broadphase_insight = !show_broadphase_insight;
				if (ImGui::MenuItem("Physics BVH Tree Visuliser")) open_phy_bvh_tree_debug = !open_phy_bvh_tree_debug;

				ImGui::EndMenu();
			}
			if (ImGui::MenuItem("Physics Entities")) open_phy_entt_win = !open_phy_entt_win;
			if (ImGui::MenuItem("Application")) open_app_win = !open_app_win;
			if (ImGui::MenuItem("Show ImGui Demo")) open_imgui_demo = !open_imgui_demo;
			ImGui::EndMenu();
		}
		if (ImGui::BeginMenu("Simulation Controls"))
		{
			auto& physics_state = mPhysicsAppSetting.StateStats();
			if (ImGui::MenuItem((physics_state.paused) ? "Resume Simulation" : "Pause Simulation", "PAUSE"))
				physics_state.paused = !physics_state.paused;
			if(ImGui::MenuItem("Single step simulation", "F10"))
				physics_state.multipleStep = 1;
			ImGui::SliderInt("N Step", &physics_state.nStep, 1, 20);
			ImGui::SameLine();
			if(ImGui::MenuItem("Multi step simulation", "Shift + F10"))
				physics_state.multipleStep = physics_state.nStep;

			//static bool b_keep_phy_settings = false;
			ImGui::Checkbox("Keep Physics settings", &mPhysicsDebugState.keepSettingsOnReset);
			if (ImGui::MenuItem("Reset world", "R")) //ResetWorld(b_keep_phy_settings);
				mPhysicsDebugState.triggerReset = true;
			if(ImGui::MenuItem("Reset Particle World"))
				mPhysicsDebugState.triggerParticleReset = true;
			float time_scale = 1.0f;
			ImGui::SliderFloat("TimeScale WIP", &time_scale, 0.0f, 1.0f, "%.1f");
			ImGui::EndMenu();
		}

		OnExpandDrawScenarioCatergory(mScenarioCatergoies);

		if(ImGui::MenuItem("Help"))
			open_help_window = !open_help_window;
		ImGui::EndMainMenuBar();
	}

	if (open_help_window)
		DrawHelpWindow(open_help_window);


	////////////////////////////////////////////
	////////////////////quick test//////////////
	////////////////////////////////////////////

	DrawApplyExternalEffectDynamicBody(nullptr);
	if(mPhysicsWorld)
	{
		if (ImGui::Begin("Quick Active Bodies Test"))
		{
			BodyID* active_bodies = mPhysicsWorld->GetActiveBodies();

			if (active_bodies)
			{
				vx::StackString<32> text("Num Bodies: ");
				text << mPhysicsWorld->GetBodies().size();
				ImGui::Text(text.Data());
				
				uint32 num_active_bodies = mPhysicsWorld->GetNumActiveBodies();
				text.Clear();
				text << "Num Active Bodies: " << num_active_bodies;
				ImGui::Text(text.Data());

				text.Clear();
				text << "Active Ratio: " << float(num_active_bodies) / float(mPhysicsWorld->GetBodies().size());
				ImGui::Text(text.Data());
				ImGui::Separator();
				for (int i = 0; i < num_active_bodies; ++i)
				{
					vx::StackString<32> label("Body_");
					label << active_bodies[i].ID() <<
						",Idx_" << active_bodies[i].Idx() <<//;
						",gen_" << active_bodies[i].Generation();
					ImGui::Text(label.Data());
				}
			}
			else
				ImGui::TextColored(ImVec4(1, 0, 0, 1), "No active bodies");

		}
		ImGui::End();
	}


	////////////////////////////////////////////
	////////////////////quick test//////////////
	////////////////////////////////////////////
	RunAllScenarioWindow();

	
	if (open_phy_debug)
	{
		if (ImGui::Begin("System Inspection", &open_phy_debug))
		{
			ImGui::BeginTabBar("#Physics Config & Debug");

			if (ImGui::BeginTabItem("Physics Settings"))
			{
				PhysicsSettingItemOverlays();
				ImGui::EndTabItem();
			}
			///// TO-DO: Later remove profiler to a seperate window
			//if (ImGui::BeginTabItem("Physics Profiler"))
			//{
			//	DrawProfileOverlay();
			//	ImGui::EndTabItem();
			//}
			if (ImGui::BeginTabItem("Renderer Resources"))
			{
				DrawUIRendererResourcesPanel();
				ImGui::EndTabItem();
			}
			ImGui::EndTabBar();
		}
		ImGui::End();
	}

	if (mNewPhyObjectSettings.openWindow) CreateNewPhysicsBodyWindow();

	if (open_phy_entt_win)
	{
		if (ImGui::Begin("Physics Entities", &open_phy_entt_win))
		{
			ImGui::BeginTabBar("#Physics Entities");

			if (ImGui::BeginTabItem("Physics - Bodies"))
			{
				if (mPhysicsWorld)
					mUI.DrawBodiesOverlayItems(mPhysicsWorld->GetBodyManager(), mPhysicsWorld);
				else
					ImGui::Text("Physics World Null!!!");
				ImGui::EndTabItem();
			}
			if (ImGui::BeginTabItem("Physics - Constraints"))
			{
				if (mPhysicsWorld)
					mUI.DrawConstraintsOverlayItems(mPhysicsWorld->GetBodyManager(), mPhysicsWorld->GetConstraints());
				else
					ImGui::Text("Physics World Null!!!");
				ImGui::EndTabItem();
			}
			if (ImGui::BeginTabItem("Physics - Particles"))
			{
				if(mParticleWorld)
					mUI.DrawParticlesOverlayItems(mParticleWorld->GetParticles());
				else
					ImGui::Text("Particle World Null!!!");
				ImGui::EndTabItem();
			}
			ImGui::EndTabBar();
		}
		ImGui::End();
	}

	if(open_imgui_demo) ImGui::ShowDemoWindow();



	if(open_app_win)
	{
		if (ImGui::Begin("Application", &open_app_win))
		{
			static bool in_ms = false;
			ImGui::Text("V-Sync: %s", (mWindow.GetVSync() ? "True" : "False"));
			ImGui::Text("frame rate: %.3f%s.",
				(in_ms) ? (float)mFrameDeltaTime * 1000 : (float)mFrameDeltaTime,
				(in_ms) ? "ms" : "s");
			ImGui::SameLine();  ImGui::Checkbox("In_ms", &in_ms);
			ImGui::SliderFloat("Time Scale", &mTimeScale, 0.0f, 1.0f);

			ImGui::BeginTabBar("#Application");


			if (ImGui::BeginTabItem("Rendering"))
			{
				if(mCurrScenario)
				{
					bool base_scenario_mouse_cast = mCurrScenario->GetAllowBaseScenarioMouseCast();
					if (ImGui::Checkbox("Allow base scenario mouse cast", &base_scenario_mouse_cast))
						mCurrScenario->SetAllowBaseScenarioMouseCast(base_scenario_mouse_cast);
				}

				auto Render_Inst_Ops = [](ERenderInstanceFlags& flags)
					{
						ImGui::CheckboxFlags("Cast Shadow", (vx::uint32*)&flags, vx::uint32(ERenderInstanceFlags::CastShadow));
						ImGui::CheckboxFlags("Receive Shadow", (vx::uint32*)&flags, vx::uint32(ERenderInstanceFlags::ReceiveShadow));
						ImGui::CheckboxFlags("Use Texture", (vx::uint32*)&flags, vx::uint32(ERenderInstanceFlags::UseTexture));
					};

				ERenderInstanceFlags* render_inst_flags[4] =
				{
					&mPhysicsRenderSettings.sphereInstanceFlags,
					&mPhysicsRenderSettings.boxInstanceFlags,
					&mPhysicsRenderSettings.capsuleInstanceFlags,
					&mPhysicsRenderSettings.planeInstanceFlags,
				};
				static constexpr const char* render_inst_names[4] =
				{
					"Sphere Render Options",
					"Box Render Options",
					"Capsule Render Options",
					"Plane Render Options",
				};
				for(int i = 0; i < 4; ++i)
				{
					//const char* name = render_inst_names[i];
					ImGui::PushID(&render_inst_names[i]);
					if (ImGui::TreeNode(render_inst_names[i]))
					{
						Render_Inst_Ops(*render_inst_flags[i]);
						ImGui::TreePop();
					}
					ImGui::PopID();
				}

				EditorImGui::Combo("Physics Text Alignment", mPhysicsRenderSettings.textAlignment, kTextAlignmentModelNames);
				ImGui::SliderFloat("Physics Text Scale", &mPhysicsRenderSettings.textScale, 0.0001, 0.1f, "%.5f");
				ImGui::Checkbox("Physics Use Dynamic Scale", &mPhysicsRenderSettings.useTestDynamicScale);
				float debug_gizmos_width = mDebugGizmos->GetLineWidth();
				if (ImGui::SliderFloat("Debug Gizmos Width", &debug_gizmos_width, 0.01f, 4.0f))
					mDebugGizmos->SetLineWidth(debug_gizmos_width);
				ImGui::Checkbox("Draw World Axes at Origin", &mPhysicsRenderSettings.drawWorldAxes);
				ImGui::DragFloat("World Axes Length", &mPhysicsRenderSettings.worldAxesLength, 0.01f, 150.0f);
				ImGui::SeparatorText("Camera");

				DrawUICameraStatePanel();

				ImGui::Separator();
				//ImGui::Spacing();
				if(ImGui::TreeNode("Lighting"))
				{
					ImGui::Text("Directional Lighting");
					DrawUI_LightingPanel();
					ImGui::Spacing();

					ImGui::SeparatorText("Shadow Data");
					DrawUI_ShadowPanel();
					ImGui::TreePop();
				}

				//ImGui::TreePop();
				ImGui::EndTabItem();
			}

			
			//bool open_impluse_win_section = ImGui::TreeNodeEx("Physics", ImGuiTreeNodeFlags_None);
			bool open_impluse_win_section = ImGui::BeginTabItem("Physics", nullptr, ImGuiTabItemFlags_None);
			if (open_impluse_win_section)
			{
				ImGui::SeparatorText("Physics Test Impluse");
				ImGui::DragFloat3("Debug angular Impluse Point", &mDebugAngularImpulse.pointA[0], 0.01f);
				ImGui::DragFloat3("Debug angular Impluse", &mDebugAngularImpulse.impluse[0], 0.01f);

				ImGui::Checkbox("Apply only angular impluse", &mDebugAngularImpulse.onlyAngularImpluse);
				ImGui::Checkbox("Apply impluse to bodies", &mDebugAngularImpulse.apply);
				ImGui::Checkbox("Draw impluse in world", &mDebugAngularImpulse.draw);

				//ImGui::TreePop();
				ImGui::EndTabItem();
			}
			mDebugAngularImpulse.draw = (open_impluse_win_section) ? mDebugAngularImpulse.draw : false;

			if (ImGui::BeginTabItem("Profiler"))
			{
				DrawProfileOverlay();
				ImGui::EndTabItem();
			}

			ImGui::EndTabBar();

			ImGui::Spacing();
			ImGui::SeparatorText("Utilies");
			ImGui::Checkbox("Show Debug Rotation", &bShowDebugRotation);
			
			ImGui::SeparatorText("Memory Usage");
			size_t _bytes = vx::sMemoryProfile.CurrentAllocBytes();
			ImGui::Text("Current Allocate Count: %llu.", vx::sMemoryProfile.CurrentAllocCount());
			if(vx::ToKilobyte(_bytes) < 1e+3)
				ImGui::Text("Current Allocate Bytes: %llu Bytes [%.2f kB | %.2f KiB].", 
					_bytes, vx::ToKilobyte(_bytes), vx::ToKibibyte(_bytes));
			else
				ImGui::Text("Current Allocate Bytes: %llu Bytes [%.2f MB | %.2f MiB].",
					_bytes, vx::ToMegabyte(_bytes), vx::ToMegabyte(_bytes));

			_bytes = vx::sMemoryProfile.allocatedBytes;
			ImGui::Text("\nTotal Allocated Calls: %llu.", vx::sMemoryProfile.allocs);
			if (vx::ToKilobyte(_bytes) < 1e+3)
				ImGui::Text("Total Allocate Bytes: %llu Bytes [%.2f kB | %.2f KiB].",
					_bytes, vx::ToKilobyte(_bytes), vx::ToKibibyte(_bytes));
			else
				ImGui::Text("Total Allocate Bytes: %llu Bytes [%.2f MB | %.2f MiB].",
					_bytes, vx::ToMegabyte(_bytes), vx::ToMegabyte(_bytes));


			_bytes = vx::sMemoryProfile.deallocatedBytes;
			ImGui::Text("\nTotal Deallocated calles: %llu .", vx::sMemoryProfile.deallocs);
			if (vx::ToKilobyte(_bytes) < 1e+3)
				ImGui::Text("Total Deallocated Bytes: %llu Bytes [%.2f kB | %.2f KiB].",
					_bytes, vx::ToKilobyte(_bytes), vx::ToKibibyte(_bytes));
			else
				ImGui::Text("Total Deallocated Bytes: %llu Bytes [%.2f MB | %.2f MiB].",
					_bytes, vx::ToMegabyte(_bytes), vx::ToMegabyte(_bytes));


		}
		ImGui::End();
	}



	if (show_broadphase_insight)
	{
		if (ImGui::Begin("Broadphase Insight", &show_broadphase_insight))
		{
			if (mPhysicsWorld)
			{
				if (ImGui::TreeNode("Broadphase Stat"))
				{
					ImGui::Text("Bodies count: %llu", static_cast<uint>(mPhysicsWorld->GetBodies().size()));
					vx::BVHBroadphase<AABB>* broad_phase;// = mPhysicsWorld->GetBVH_AABB_Broadphase();
					{
						auto* _v = mPhysicsWorld->GetBroadphase();
						if (_v->Type() != vx::EBroadphaseType::BVH)
							return;
						broad_phase = reinterpret_cast<vx::BVHBroadphase<AABB>*>(_v);
					}
					if (broad_phase != nullptr)
					{
						auto stat = broad_phase->GetTreeStat().stats;
						ImGui::Text(stat->AsString().Data());
						float balance_due_depth = broad_phase->GetTreeStat().tree->ComputeDepthImbalance();
						ImGui::TextColored(((balance_due_depth < 0.5) ? ImVec4(0.0, 1.0, 0.0, 1.0) :
							((balance_due_depth > 0.8) ? ImVec4(1.0, 0.0, 0.0, 1.0) :
								ImVec4(1.0, 1.0, 0.0, 1.0))), "Imbalance due aver depth %f", balance_due_depth);
						ImGui::Text("Average transversal Jump %f", broad_phase->GetTreeStat().tree->ComputeAverageJump());
					}
					ImGui::TreePop();
				}

				ImGui::Separator();
				if (ImGui::TreeNodeEx("Broadphase Pair", ImGuiTreeNodeFlags_DefaultOpen))
				{
					uint32 bp_count = mPhysicsWorld->GetBroadphasePairsCount();
					ImGui::Text("Pair count: %llu", static_cast<uint>(bp_count));
					int idx = 0;

					const vx::BodyManager& body_manager = mPhysicsWorld->GetBodyManager();
					const auto* bps = mPhysicsWorld->GetBroadphasePairsPtr();
					for (const BroadphasePair* bp = bps, *bp_end = bps + bp_count; bp < bp_end; ++bp)
					{
						const auto& p_a = (*bp).a, p_b = (*bp).b;
						ImGui::PushID(idx);
						//ImGui::Text("Pair %d: [%s] with [%s].", idx++, p_a->mDebugName.c_str(), p_b->mDebugName.c_str());
						ImGui::Text("Pair %d: [%s] with [%s].", idx++, 
							body_manager.GetBodyDebugName(p_a->GetID()),
							body_manager.GetBodyDebugName(p_b->GetID()));
						ImGui::PopID();
					}
					ImGui::TreePop();
				}
			}
			else
				ImGui::Text("Physics World is null!!!");
		}
		ImGui::End();
	}


	if (open_phy_bvh_tree_debug && mPhysicsWorld && mPhysicsWorld->GetBroadphase()) DrawBVHNodesOverlay(open_phy_bvh_tree_debug);
}

void Application::OnDrawBodiesOverlays()
{






	if (ImGui::Begin("Rigibodies"))
	{
		auto& physics_bodies = mPhysicsWorld->GetBodies();
		for (auto& body : physics_bodies)
		{
			ImGui::PushID(&body);
			vx::Mat44 tranform = body.ComputeWorldTransform();
			//Vec3 pos = tranform[3];
			Vec3 pos = tranform.GetTranslation();
			if (ImGui::DragFloat3("mPosition", &pos[0], 0.1f))
			//if (ImGui::DragFloat3("mPosition", &tranform.GetTranslation()[0], 0.1f))
			{
				tranform.SetTranslation(pos);
				body.SetWorldTransform(tranform);
			}

			//ImGui::Text("mAcceleration: %s", particle.mAcceleration.ToString().c_str());
			//ImGui::Text("mAccumulatedForce: %s", particle.mAccumlatedForce.ToString().c_str());
			//ImGui::Text("mVelocity: %s", particle.mVelocity.ToString().c_str());


			//ImGui::SliderFloat("mDamping", &particle.mDamping, 0.0f, 1.0f);
			//ImGui::SliderFloat("mMass", &particle.mMass, 0.0f, 200.0f, "%.1f");
			//ImGui::SliderFloat("mRadius", &particle.mRadius, 0.0f, 1.0f);
			ImGui::PopID();
		}
	}
	ImGui::End();

}

//void Application::DrawProfileOverlay()
//{
//#ifdef VPHX_ENABLE_PROFILING
//	auto& profiles = VPHX::Profiler::ProfilerCollector::Instance().GetProfiles();
//
//	if (ImGui::Begin("Physics Profiler"))
//	{
//		for (auto& [name, data] : profiles)
//		{
//			ImVec4 col = data.avgMs < 0.1 ? ImVec4(0, 1, 0, 1)
//				: data.avgMs < 0.5 ? ImVec4(1, 1, 0, 1)
//				: ImVec4(1, 0, 0, 1);
//
//			ImGui::TextColored(col, "%-40s last: %.3f avg: %.3f max: %.3f (samples: %llu)",
//				name.data(), data.lastMs,
//				data.avgMs, data.maxMs, data.samples);
//		}
//	}
//	ImGui::End();
//#endif // VPHX_ENABLE_PROFILING
//
//}

//
//void Application::DrawProfileOverlay()
//{
//#ifdef VPHX_ENABLE_PROFILING
//
//	if (ImGui::Begin("Physics Profiler"))
//	{
//		static std::unordered_map<std::string_view, std::vector<float>> profile_samples;
//		static int frame_idx = 0;
//		constexpr int k_max_samples = 200;
//
//		auto& profiles = VPHX::Profiler::ProfilerCollector::Instance().GetProfiles();
//		for (auto& [name, data] : profiles)
//		{
//			//ImVec4 col = data.avgMs < 0.1 ? ImVec4(0, 1, 0, 1)
//			//	: data.avgMs < 0.5 ? ImVec4(1, 1, 0, 1)
//			//	: ImVec4(1, 0, 0, 1);
//
//			//ImGui::TextColored(col, "%-40s last: %.3f avg: %.3f max: %.3f (samples: %llu)",
//			//	name.data(), data.lastMs,
//			//	data.avgMs, data.maxMs, data.samples);
//
//			auto& samples = profile_samples[name];
//			if (samples.size() < k_max_samples)
//				samples.resize(k_max_samples, 0.0f);
//
//			samples[frame_idx] = data.lastMs;
//		}
//
//		ImGui::SeparatorText("Perfomance Graph");
//		ImDrawList* draw_list = ImGui::GetWindowDrawList();
//		ImVec2 p = ImGui::GetCursorScreenPos();
//		ImVec2 size = ImGui::GetContentRegionAvail();
//
//		float padding = 20.0f;
//		ImVec2 graph_pos = ImVec2(p.x + padding, p.y + padding);
//		ImVec2 graph_size = ImVec2(size.x - padding * 2.0f, size.y - padding * 2.0f);
//
//		auto add_im_vec2 = [](const ImVec2& a, const ImVec2& b) {
//			ImVec2 result = a;
//			result.x += b.x;
//			result.y += b.y;
//			return result;
//			};
//
//		//draw a background
//		//draw_list->AddRectFilled(p, ImVec2(p.x + size.x, p.y + size.y), IM_COL32(30, 30, 30, 255));
//		draw_list->AddRectFilled(graph_pos, add_im_vec2(graph_pos, graph_size), IM_COL32(20, 20, 20, 255));
//		float right_label_padding = 100.0f;
//		graph_size.x -= right_label_padding;
//		draw_list->AddRect(graph_pos, add_im_vec2(graph_pos, graph_size), IM_COL32(80, 80, 80, 255));
//
//		//Plot each profile as a line with its own colour
//		int colour_idx = 0;
//		for (auto& [name, samples] : profile_samples)
//		{
//			ImU32 col = ImColor::HSV(float(colour_idx++) / float(profile_samples.size()), 0.8f, 0.9f);
//			for (int i = 1; i < k_max_samples; i++)
//			{
//				float x0 = p.x + (float)(i - 1) / (k_max_samples - 1) * graph_size.x;
//				float x1 = p.x + (float)(i) / (k_max_samples - 1) * graph_size.x;
//				float y0 = p.y + graph_size.y -  samples[(frame_idx + i - 1) % k_max_samples] * 100.0f;
//				float y1 = p.y + graph_size.y -  samples[(frame_idx + i) % k_max_samples] * 100.0f;
//				draw_list->AddLine(ImVec2(x0, y0), ImVec2(x1, y1), col);
//
//				int last_index = (frame_idx + k_max_samples - 1) % k_max_samples;
//				float x_lable = p.x + graph_size.x + 5.0f;
//				float y_lable = p.y + graph_size.y - samples[last_index] * 100.0f;
//				ImVec2 text_pos(x_lable, y_lable - ImGui::GetTextLineHeight() * 0.5f);
//				draw_list->AddText(ImGui::GetFont(), ImGui::GetFontSize(), text_pos, col, name.data());
//				draw_list->AddCircleFilled(ImVec2(p.x + graph_size.x, y_lable), 3.0f, col);
//			}
//		}
//
//		ImGui::Dummy(graph_size); //Advance cursor
//		frame_idx = (frame_idx + 1) % k_max_samples;
//	}
//	ImGui::End();
//#endif // VPHX_ENABLE_PROFILING
//
//}

void Application::PhysicsSettingItemOverlays()
{
	if (!mPhysicsWorld)
		return;

	///quick testing other window is get crowded 
	auto Contact_Solver_Win = [&](bool* p_open)
	{
		if (ImGui::Begin("Contact Constraint Solver", p_open))
		{
			auto& solver_stat = mPhysicsWorld->GetContactConstraintSolverStats();
			ImGui::Text("Number of Contacts: %d", solver_stat.numContactConstraints);
			ImGui::Text("Number of Persistent Contacts: %d", solver_stat.numPersistentContact);

			ImGui::Text("Total Kinetic Work: %d", solver_stat.totalKineticWork);
			ImGui::Text("Total Step Kinetic Work: %d", solver_stat.totalStepKineticWork);
			ImGui::Text("Max Attained Kinetic Work: %d", solver_stat.maxAttainedKineticWork);

			ImGui::Text("Total Step Work Loss: %d", solver_stat.totalStepWorkLoss);
			ImGui::Text("Total Step Work Gain: %d", solver_stat.totalStepWorkGain);

			ImGui::End();
		}
	};

	static bool open_contact_solver_win = false;
	if (open_contact_solver_win)
		Contact_Solver_Win(&open_contact_solver_win);

	//colour 
	auto imgui_colour_edit = [](const char* label, vx::Colour& c) {
		vx::Vec4 v = c.ToVec4();
		bool changed = ImGui::ColorEdit3(label, &v[0]);
		if (changed)
			c.ToColour(&v[0], true);
		return changed;
		};

	auto imgui_colour_edit4 = [](const char* label, vx::Colour& c) {
		vx::Vec4 v = c.ToVec4();
		bool changed = ImGui::ColorEdit4(label, &v[0]);
		if (changed)
			c.ToColour(&v[0], true);
		return changed;
		};

	ImGui::SeparatorText("APP SETTING");

	static bool format_KiB = true;
	float status_kB = (format_KiB) ? vx::ToKibibyte(mPhysicsWorld->mScratchAllocator->Usage()) :
									 vx::ToMebibyte(mPhysicsWorld->mScratchAllocator->Usage());
	float size_kB = (format_KiB) ? vx::ToKibibyte(mPhysicsWorld->mScratchAllocator->Size()) :
											vx::ToMebibyte(mPhysicsWorld->mScratchAllocator->Size());
	vx::StackString<32> text;
	text << status_kB << "/" << size_kB << ((format_KiB) ? " KiB" : " MiB");
	float ratio = status_kB / size_kB;
	ImGui::ProgressBar(ratio, ImVec2(0.0f, 0.0f), text.Data());
	ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
	ImGui::Text("Scratch Allocation");
	ImGui::Checkbox("Format KiB", &format_KiB);

	static int freq_idx = (int)std::log2((float)mPhysicsAppSetting.Rate() / 30.0f);
	if (ImGui::Combo("Physics Freq (Hz)", &freq_idx, "30 Hz\0""60 Hz\0""120 Hz\0""240 Hz\0"))
		mPhysicsAppSetting.SetFrequency((30 * (1 << freq_idx)));
	static bool in_ms = false;
	//ImGui::Text("Fixed Time Step %f %s.", (in_ms) ? (mPhysicsAppSetting.FixedTimeStep() * to_ms, "ms") : (mPhysicsAppSetting.FixedTimeStep(), "s"));
	ImGui::Text("Fixed Time Step %.3f%s.", 
		(in_ms) ? (mPhysicsAppSetting.FixedTimeStep() * 1000) : mPhysicsAppSetting.FixedTimeStep(), 
		(in_ms) ? "ms":"s");
	ImGui::SameLine(); ImGui::Checkbox("In ms", &in_ms);
	ImGui::SliderInt("Max Physics Sub steps", &mPhysicsAppSetting.mMaxSubStep, 1, 8);

	auto& physics_state = mPhysicsAppSetting.StateStats();
	ImGui::Separator();
	ImGui::Text("Sim: ");
	ImGui::SameLine(); if(ImGui::Button((physics_state.paused)?"Paused (Play)":"Running (Pause)"))
		physics_state.paused = !physics_state.paused;
	ImGui::SliderInt("N Step", &physics_state.nStep, 1, 20);
	vx::StackString<16> button_text("Step ");
	button_text << physics_state.nStep;
	///std::string button_text = "Step " + std::to_string(physics_state.nStep);
	ImGui::SameLine(); if (ImGui::Button(button_text.Data())) physics_state.multipleStep = physics_state.nStep;

	ImGui::Separator();

	if (ImGui::TreeNode("Physics Frame State"))
	{
		const PhysicsAppSetting::State& curr_state = mPhysicsAppSetting.StateStats();
		ImGui::Text("Accumulated frame time %.3fs.", curr_state.accumulator);
		ImGui::Text("Frame time %.3fs.", curr_state.frameTime);
		ImGui::Text("Max attained frame time %.3fs.", curr_state.maxAttainedTime);
		ImGui::Text("Substep in frame %d.", curr_state.subSteps);
		ImGui::Text("Max attained substep %d.", curr_state.maxAttainedSubStep);
		ImGui::TreePop();
	}

	ImGui::Separator();

	auto& phy_settings = mPhysicsWorld->GetSettings();

	if (ImGui::TreeNodeEx("SETTINGS", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::DragFloat3("Gravity", &phy_settings.gravity[0]);
		ImGui::SliderFloat("Gravity Scale", &phy_settings.gravityScale, 0.0f, 1.0f, "%.2f");


		if (ImGui::TreeNodeEx("COLLISION"))
		{
			ImGui::Checkbox("Force BVH Rebuild", &phy_settings.forceBVHRebuild);
			ImGui::Checkbox("Rebuild BVH SAH", &phy_settings.collision.BVH_rebuild_SAH);
			if (ImGui::SliderFloat("Collision Bounds Margin (m)", &phy_settings.collision.boundsMargin, 0.01f, 0.6f))
				mPhysicsWorld->SetBroadphaseNodeBoundThreshold(phy_settings.collision.boundsMargin);
			ImGui::SliderFloat("Imbalance ratio treshold rebuild", &phy_settings.collision.rebuildBVH_ImbalanceRatioTreshold, 0.0f, 1.0f);
			ImGui::Checkbox("Use New Manifold pt", &vx::ManifoldPoint::kUseNewManifoldPt);
			ImGui::Checkbox("Debug Box - Box contacts", &phy_settings.drawSettings.drawAABBContactManifoldInFrame);
			ImGui::Checkbox("Debug Box - Box contacts with plane", &phy_settings.drawSettings.drawAABBContactManifoldInFrameWcPlane);
			if (ImGui::TreeNode("Narrowphase Stats"))
			{
				//if(auto& narrow_stats = mPhysicsWorld->GetNarrowphaseStats())
				const auto& narrow_stats = mPhysicsWorld->GetNarrowphaseStats();
				
				ImGui::Text("Number of pair received: %d", narrow_stats.numPairReceived);
				ImGui::Text("Number of contact pairs: %d", narrow_stats.numContactPair);
				ImGui::Text("Max attained contact pairs: %d", narrow_stats.maxAttainedContactPair);
				
				ImGui::TreePop();
			}

			ImGui::TreePop();
		}


		if (ImGui::TreeNodeEx("SOLVER"))
		{
			ImGui::Checkbox("Enable Solver", &phy_settings.solver.enable);
			ImGui::Checkbox("Enable Contact Solver", &phy_settings.solver.enableContact);
			ImGui::Checkbox("Enable Warm Start", &phy_settings.solver.warmstart);
			ImGui::Checkbox("Ensure Contact Manifold Consistent", &phy_settings.collision.consistentManifold);
			ImGui::SliderInt("Contact Constraint Interations", &phy_settings.solver.velocityIterations, 0, 25);
			ImGui::SliderInt("Contact Position Interations", &phy_settings.solver.positionIterations, 0, 10);
			int v = (int)phy_settings.solver.restitutionCombineMode;
			if (ImGui::Combo("Restitution Combine Mode", &v, vx::CoefficientCombineModeNames))
			{
				phy_settings.solver.restitutionCombineMode = (vx::ECombineMode)v;
				mPhysicsWorld->SetRestitutionCombineMode(phy_settings.solver.restitutionCombineMode);
			}
			v = (int)phy_settings.solver.frictionCombineMode;
			if (ImGui::Combo("Friction Combine Mode", &v, vx::CoefficientCombineModeNames))
			{
				phy_settings.solver.frictionCombineMode = (vx::ECombineMode)v;
				mPhysicsWorld->SetFrictionCombineMode(phy_settings.solver.frictionCombineMode);
			}

			ImGui::SliderFloat("Restitution Threshold", &phy_settings.solver.restitutionThreshold, 0.01f, 2.0f);
			ImGui::SliderFloat("Friction Threshold (m/s)", &phy_settings.solver.frictionThreshold, 0.0f, 0.001f, "%.6f");

			if (ImGui::TreeNode("Position Contact Settings"))
			{
				ImGui::SliderFloat("Slop", &phy_settings.solver.positionCorrectionSlop, 0, 1, "%.2f");
				ImGui::SliderFloat("Bias", &phy_settings.solver.baumgarte, 0, 1, "%.1f");
				ImGui::SliderFloat2("Limit",
					&phy_settings.solver.positionCorrectionGlobalLimits[0],
					phy_settings.solver.positionCorrectionGlobalLimits[0] - 0.2f,
					phy_settings.solver.positionCorrectionGlobalLimits[1] + 0.2f, "%.1f");
				ImGui::TreePop();
			}

			ImGui::Checkbox("Contact Constraint Solver Window", &open_contact_solver_win);
			ImGui::TreePop();
		}


		if (ImGui::TreeNodeEx("SLEEPING"))
		{
			ImGui::Checkbox("Enable Sleeping", &phy_settings.sleeping.enable);
			ImGui::SliderFloat("Sleep Velocity Threshold", &phy_settings.sleeping.velocityThreshold, 0.0f, 1.0f);
			ImGui::SliderFloat("Sleep Angular Threshold", &phy_settings.sleeping.angularThreshold, 0.0f, 1.0f);
			ImGui::SliderFloat("Sleep Time Threshold", &phy_settings.sleeping.timeThreshold, 0.0f, 1.0f);

			ImGui::TreePop();
		}


		ImGui::TreePop();
	}

	
	vx::DrawSettings& draw_settings = phy_settings.drawSettings;
	if(ImGui::TreeNodeEx("DEBUG DRAW", ImGuiTreeNodeFlags_DefaultOpen))
	{
		if (ImGui::TreeNodeEx("Body", ImGuiTreeNodeFlags_DefaultOpen))
		{
			EditorImGui::Combo("Body Colour mode", draw_settings.bodyColourMode, vx::BodyColourModeLabels);
			ImGui::Checkbox("draw AABB", &draw_settings.drawAABB);
			ImGui::Checkbox("draw OBB", &draw_settings.drawOBB);
			ImGui::Checkbox("draw Bodies as Solid Mesh", &draw_settings.drawBodiesAsSolid);
			ImGui::Checkbox("draw Bodies Principal Axes", &draw_settings.drawBodiesPrincipalAxes);
			ImGui::Checkbox("draw Bodies Debug Inertia", &draw_settings.drawDebugInertia);
			ImGui::Checkbox("draw Bodies Motion Velocity", &draw_settings.drawBodiesVelocities);
			ImGui::Checkbox("draw Bodies Mass Text", &draw_settings.drawBodiesMassText);
			ImGui::Checkbox("draw Shape corners", &draw_settings.drawShapeOrientedBoundCorners);
			ImGui::TreePop();
		}

		if (ImGui::TreeNodeEx("World Query"))
		{
			ImGui::Checkbox("draw Broadphase Walked Nodes", &draw_settings.drawWalkedTreeQuery);
			ImGui::TreePop();
		}

		if (ImGui::TreeNodeEx("Non Contact Constraint"))
		{
			NonContactConstraintDrawSettings& constraint_draw = draw_settings.nonContactConstraintDrawSettings;
			ImGui::Checkbox("draw constraints", &constraint_draw.drawConstraints);
			ImGui::SliderFloat("draw anchor size", &constraint_draw.anchorSize, 0.01f, 2.0f);
			ImGui::Checkbox("draw constraint bounds", &constraint_draw.drawConstraintBounds);
			ImGui::Checkbox("draw active bounds", &constraint_draw.drawActiveBounds);
			ImGui::Checkbox("draw velocity solve bounds", &constraint_draw.drawVelocitySolveBounds);
			ImGui::Checkbox("draw position solve bounds", &constraint_draw.drawPositionSolveBounds);
			ImGui::TreePop();
		}

		if (ImGui::TreeNodeEx("Broadphase"))
		{
			bool _flag = vx::Contains(draw_settings.braodphaseFlags, vx::EBroadphaseDrawFlag::LeafNodes) &&
				vx::Contains(draw_settings.braodphaseFlags, vx::EBroadphaseDrawFlag::InternalNodes);
			if (ImGui::Checkbox("draw BVH Nodes", &_flag))
				draw_settings.braodphaseFlags ^= vx::EBroadphaseDrawFlag::All;
			_flag = vx::Contains(draw_settings.braodphaseFlags, (vx::EBroadphaseDrawFlag::LeafNodes | ~vx::EBroadphaseDrawFlag::InternalNodes));
			if (ImGui::Checkbox("draw only BVH Leaf Nodes", &_flag))
				draw_settings.braodphaseFlags ^= vx::EBroadphaseDrawFlag::LeafNodes;
			ImGui::TreePop();
		}


		if (ImGui::TreeNodeEx("Narrowphase"))
		{
			ImGui::Checkbox("draw Contacts", &draw_settings.drawContacts);
			ImGui::SameLine(); ImGui::SliderFloat("contact size", &draw_settings.drawContactPointSize, 0.01f, 1.5f, "%.2f");
			ImGui::Checkbox("draw Contacts Normals", &draw_settings.drawContactsNormals);
			ImGui::Checkbox("draw Contacts Normals With Penetration", &draw_settings.drawContactsNormalsWithPeneration);
			ImGui::Checkbox("draw Contacts Scale Normal Penetration", &draw_settings.drawContactsScaleNormalPeneration);
			if (draw_settings.drawContactsScaleNormalPeneration)
			{
				ImGui::SameLine();
				ImGui::SliderFloat("scale", &draw_settings.drawContactNormalSize, 0.5f, 5.0f, "%.2f");
			}
			ImGui::Checkbox("Draw Colliding Pair Ref And Inc", &draw_settings.drawCollidingPairRefAndInc);
			ImGui::TreePop();
		}

		if (ImGui::TreeNodeEx("Solver"))
		{
			ImGui::Checkbox("draw Contact Constraint Solver TBNs", &draw_settings.drawContactConstraintSolverTBNs);
			ImGui::TreePop();
		}
		ImGui::TreePop();
	}


	ImGui::Spacing();
	if (ImGui::TreeNodeEx("COLOUR SCHEME"))
	{
		imgui_colour_edit4("Dynamic Colour", draw_settings.dynamicColour);
		imgui_colour_edit4("Static Colour", draw_settings.staticColour);
		imgui_colour_edit4("Sleeping Colour", draw_settings.sleepingColour);
		imgui_colour_edit4("Ground Colour", draw_settings.groundColour);
		imgui_colour_edit("Colliding pair Ref Colour", draw_settings.collidingPairRefColour);
		imgui_colour_edit("Colliding pair Inc Colour", draw_settings.collidingPairIncColour);
		imgui_colour_edit("Contact Colour", draw_settings.contactColour);
		imgui_colour_edit("Contact Wire Colour", draw_settings.contactWireColour);
		imgui_colour_edit("Contact point colour", draw_settings.drawContactPointColour);
		imgui_colour_edit("Contact normal colour", draw_settings.drawContactNormalsColour);
		imgui_colour_edit("Shape corners colour", draw_settings.drawShapeCornersColour);
		imgui_colour_edit("Body Debug Inertia colour", draw_settings.drawDebugInertiaColour);

		imgui_colour_edit("Shape Collider Wire Colour", draw_settings.shapeColliderWireColour);
		imgui_colour_edit("AABB Colour", draw_settings.aabbColour);
		imgui_colour_edit("bvh Node Colour", draw_settings.bvhNodeColour);
		imgui_colour_edit("Dynamic-Static Collision", draw_settings.dynamicStaticCollisionColour);
		imgui_colour_edit("Narrow phase Collision", draw_settings.narrowPhaseColour);
		imgui_colour_edit("Broad phase Collision", draw_settings.broadPhaseColour);
		imgui_colour_edit("Neural Collision", draw_settings.neuralPhaseColour);
		imgui_colour_edit("Linear Velocity Draw Colour", draw_settings.bodyLinearVelocityCol);
		imgui_colour_edit("Angular Velocity Draw Colour", draw_settings.bodyAngularVelocityCol);
		ImGui::TreePop();
	}
}

void Application::DrawProfileOverlay()
{
#ifdef VX_ENABLE_PROFILING
	using namespace vx::Profiler;

	constexpr int MAX_SAMPLES = 200;
	static std::unordered_map<std::string_view, std::vector<float>> profile_samples;
	static int frame_index = 0;

	auto& profiles = ProfilerCollector::Instance().GetProfiles();


	auto Filter_Pass = [&](const std::string_view name, const char* filter)
		{
			if (!filter || filter[0] == '\0')
				return true;

			std::string f(filter);
			std::transform(f.begin(), f.end(), f.begin(), ::tolower);

			std::stringstream ss(f);
			std::string or_chunk;

			while (std::getline(ss, or_chunk, '|'))
			{
				or_chunk.erase(0, or_chunk.find_first_not_of(" \t"));
				or_chunk.erase(or_chunk.find_last_not_of(" \t") + 1);

				if (or_chunk.empty())
					continue;

				std::stringstream and_split(or_chunk);
				std::string token;
				bool all_match = true;

				while (and_split >> token)
				{
					//if(name.empty())


					std::string hay(name);
					std::string needle = token;
					std::transform(hay.begin(), hay.end(), hay.begin(), ::towlower);
					std::transform(needle.begin(), needle.end(), needle.begin(), ::towlower);

					if (hay.find(needle) == std::string::npos)
					{
						all_match = false;
						break;
					}

					//if (hay.find(needle) != std::string::npos)
					//	return true;
				}

				if (all_match)
					return true;
			}

			return false;
			//while (ss >> or_chunk)
			//	if (name.find(or_chunk) == std::string_view::npos)
			//		return false;
			//return true;
			////return std::strlen(filter) > 0 && name.find(filter) == std::string_view::npos;
		};


	if (ImGui::TreeNode("Physics Stat") && mPhysicsWorld)
	{
		ImGui::Text("Bodies count: %llu", static_cast<uint>(mPhysicsWorld->GetBodies().size()));
		vx::BVHBroadphase<AABB>* broad_phase;// = mPhysicsWorld->GetBVH_AABB_Broadphase();
		{
			auto* _v = mPhysicsWorld->GetBroadphase();
			if (_v->Type() != vx::EBroadphaseType::BVH)
				return;
			broad_phase = static_cast<vx::BVHBroadphase<AABB>*>(_v);
		}
		if (broad_phase)
		{
			auto stat = broad_phase->GetTreeStat().stats;
			ImGui::Text(stat->AsString().Data());
			float balance_due_depth = broad_phase->GetTreeStat().tree->ComputeDepthImbalance();
			ImGui::TextColored(((balance_due_depth < 0.5) ? ImVec4(0.0, 1.0, 0.0,1.0) : 
				((balance_due_depth > 0.8) ? ImVec4(1.0, 0.0, 0.0, 1.0) : 
					ImVec4(1.0, 1.0, 0.0, 1.0))), "Imbalance due aver depth %f", balance_due_depth);
			ImGui::Text("Average transversal Jump %f", broad_phase->GetTreeStat().tree->ComputeAverageJump());
		}
		ImGui::TreePop();
	}

	static char filter[128] = "";
	if (ImGui::InputText("Filter", filter, 128))
	{
		for (auto& [profile, samples] : profile_samples)
			if (!Filter_Pass(profile, filter))
				std::fill(samples.begin(), samples.end(), 0.0f);
	}
	ImGui::SetItemTooltip("Filter supports OR '|' and AND ' '.");

	bool profile_timer_open = ImGui::TreeNodeEx("Profile Timers", ImGuiTreeNodeFlags_DefaultOpen);

	// Ensure sample buffers exist and update them from profiles
	for (const auto& [name, data] : profiles)
	{
		if (!Filter_Pass(name, filter))
			continue;

		if(profile_timer_open)
		{
			ImVec4 col = data.avgMs < 0.1 ? ImVec4(0, 1, 0, 1)
				: data.avgMs < 0.5 ? ImVec4(1, 1, 0, 1)
				: ImVec4(1, 0, 0, 1);

		
				ImGui::TextColored(col, "%-40s last: %.4f avg: %.4f max: %.4f (samples: %llu)",
					name.data(), data.lastMs,
					data.avgMs, data.maxMs, data.samples);
		}

		auto it = profile_samples.find(name);
		if (it == profile_samples.end())
		{
			std::vector<float> buf(MAX_SAMPLES, 0.0f);
			auto res = profile_samples.emplace(name, std::move(buf));
			it = res.first;
		}
		// write latest sample into buffer at current index
		it->second[frame_index % MAX_SAMPLES] = static_cast<float>(data.lastMs);
	}

	if(profile_timer_open)
		ImGui::TreePop();

	static float padding = 12.0f;
	static float right_label_padding = 450.0f; // space for labels on right
	static float top_header_offset = 6.0f;
	// Draw Y-axis ticks/labels
	static int num_ticks = 3; // 0%, 25%, 50%, 75%, 100%
	static bool avoid_label_colliding = false;
	static float sample_max_range_min = 0.1f;
	static float sample_max_range_max = 0.5f;
	// Header / small stats table
	if(ImGui::TreeNodeEx("Profile Graph", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::Separator();

		// Reserve area and padding
		ImVec2 avail = ImGui::GetContentRegionAvail();
		const float top_header = ImGui::GetTextLineHeight() + top_header_offset;

		// Build graph rect (leave space for right labels)
		ImVec2 p = ImGui::GetCursorScreenPos();
		p.x += 40.0f;
		ImVec2 graph_pos = ImVec2(p.x + padding, p.y + padding + top_header);
		ImVec2 graph_size = ImVec2(
			std::max(64.0f, avail.x - padding * 2.0f - right_label_padding),
			std::max(80.0f, avail.y - padding * 2.0f - top_header)
		);

		ImDrawList* drawList = ImGui::GetWindowDrawList();

		// Background + border
		ImU32 bg = IM_COL32(22, 22, 22, 230);
		ImU32 border = IM_COL32(80, 80, 80, 200);
		drawList->AddRectFilled(graph_pos, ImVec2(graph_pos.x + graph_size.x, graph_pos.y + graph_size.y), bg);
		drawList->AddRect(graph_pos, ImVec2(graph_pos.x + graph_size.x, graph_pos.y + graph_size.y), border);


		// Compute a simple max for auto-scaling
		static float smooth_max_ms = 1.0f;
		float instant_max = 0.00001f;
		for (auto& kv : profile_samples)
			for (float v : kv.second)
				instant_max = std::max(instant_max, v);


		float rise_rate = 0.2f;
		float fall_rate = 0.05f;
		if (instant_max > smooth_max_ms)
			smooth_max_ms = smooth_max_ms * (1.0f - rise_rate) + instant_max * rise_rate;
		else
			smooth_max_ms = smooth_max_ms * (1.0f - fall_rate) + instant_max * fall_rate;

		float max_sample = smooth_max_ms;

		// give a little headroom
		float scale_y = (max_sample > 0.0f) ? (graph_size.y / (max_sample * 1.2f)) : (graph_size.y / 1.0f);
		scale_y = std::max(scale_y, 50.0f); // ensure visible scaling: pixels per ms



		// Draw each profile line. Use deterministic color per name (hash).
		std::vector<float> used_label_ys;
		used_label_ys.reserve(profile_samples.size());
		int colour_index = 0;
		for (auto& [name, samples] : profile_samples)
		{
			if (!Filter_Pass(name, filter))
				continue;

			// Stable color via hash -> HSV
			uint32_t h = std::hash<std::string_view>{}(name);
			float hue = (h & 0xFFFF) / float(0xFFFF);
			ImU32 col = ImColor::HSV(hue, 0.7f, 0.9f);

			// Draw polyline
			for (int i = 1; i < MAX_SAMPLES; ++i)
			{
				int si0 = (frame_index + i - 1) % MAX_SAMPLES;
				int si1 = (frame_index + i) % MAX_SAMPLES;
				float v0 = samples[si0];
				float v1 = samples[si1];

				float x0 = graph_pos.x + (i - 1) / float(MAX_SAMPLES - 1) * graph_size.x;
				float x1 = graph_pos.x + (i) / float(MAX_SAMPLES - 1) * graph_size.x;
				float y0 = graph_pos.y + graph_size.y - (v0 * scale_y);
				float y1 = graph_pos.y + graph_size.y - (v1 * scale_y);

				// clamp inside graph rect
				y0 = vx::VxClamp(y0, graph_pos.y, graph_pos.y + graph_size.y);
				y1 = vx::VxClamp(y1, graph_pos.y, graph_pos.y + graph_size.y);

				drawList->AddLine(ImVec2(x0, y0), ImVec2(x1, y1), col, 1.5f);
			}

			// Last sample marker + label
			int last_idx = (frame_index + MAX_SAMPLES - 1) % MAX_SAMPLES;
			float last_v = samples[last_idx];
			float label_y = graph_pos.y + graph_size.y - (last_v * scale_y);
			label_y = vx::VxClamp(label_y, graph_pos.y, graph_pos.y + graph_size.y);

			// simple collision-avoid for labels (push up if too close)
			if (avoid_label_colliding)
			{
				for (float y : used_label_ys)
				{
					if (fabsf(y - label_y) < 12.0f)
						label_y -= 12.0f;
				}
			}
			used_label_ys.push_back(label_y);

			// dot and text on right
			ImVec2 dot_pos(graph_pos.x + graph_size.x, label_y);
			drawList->AddCircleFilled(dot_pos, 3.0f, col);
			ImVec2 textPos(graph_pos.x + graph_size.x + 6.0f, label_y - ImGui::GetTextLineHeight() * 0.5f);
			drawList->AddText(textPos, col, std::string(name).c_str());

			++colour_index;
		}

		// Advance frame index (do this once per frame)
		frame_index = (frame_index + 1) % MAX_SAMPLES;

		// Reserve the ImGui item space so layout doesn't collapse
		ImGui::Dummy(ImVec2(graph_size.x + right_label_padding + padding * 2.0f, graph_size.y + padding * 2.0f + top_header));
		// Draw Y-axis ticks/labels & grid 
		//static int numTicks = 5; // 0%, 25%, 50%, 75%, 100%
		for (int i = 0; i <= num_ticks; ++i)
		{
			float t = float(i) / float(num_ticks);
			float y = graph_pos.y + graph_size.y - t * graph_size.y; // invert y for ImGui coords
			drawList->AddLine(
				ImVec2(graph_pos.x - 4, y),
				ImVec2(graph_pos.x, y),
				IM_COL32(180, 180, 180, 255)
			);

			float msVal = t * max_sample * 1.2f; // 1.2 headroom
			char buf[16];
			snprintf(buf, sizeof(buf), "%.2fms", msVal);
			drawList->AddText(ImVec2(graph_pos.x - 50, y - ImGui::GetTextLineHeight() * 0.5f), IM_COL32(200, 200, 200, 255), buf);

			////grid 

			//drawList->AddLine(
			//	ImVec2(graph_pos.x, graph_pos.y + t * graphSize.y),
			//	ImVec2(graph_pos.x + graphSize.x, graph_pos.y + t * graphSize.y),
			//	IM_COL32(60, 60, 60, 120));

			drawList->AddLine(
				ImVec2(graph_pos.x, y),
				ImVec2(graph_pos.x + graph_size.x, y),
				IM_COL32(60, 60, 60, 120));
		}
		ImGui::TreePop();


	}
	if (ImGui::TreeNode("Settings"))
	{
		ImGui::DragFloat("Padding", &padding, 0.1f, 0.0f, 5.0f);
		ImGui::DragFloat("Right Label Padding", &right_label_padding, 1.0f, 140.0f, 500.0f);
		ImGui::DragFloat("Top Header Offset", &top_header_offset, 0.1f, 0.0f, 15.0f);
		ImGui::SliderInt("Tick", &num_ticks, 1, 6);
		ImGui::Checkbox("Avoid Label Colliding", &avoid_label_colliding);
		ImGui::SliderFloat("Sample Max Range Min", &sample_max_range_min, 0.01f, 0.2f);
		ImGui::SliderFloat("Sample Max Range Min", &sample_max_range_max, 0.1f, 2.0f);
		ImGui::TreePop();
	}
#endif // VPHX_ENABLE_PROFILING
}

void Application::DrawUICameraStatePanel()
{
	auto& cam_props = mCamera.GetProperties();
	ImGui::SliderFloat("FovY", &cam_props.fovY, 0.0f, 120.0f, "%.1f");
	static float clipping_plane_prop_speed = 0.05f;
	if (ImGui::DragFloat("Near", &cam_props.zNear, clipping_plane_prop_speed, 0.0f))
		cam_props.zNear = VxMax(cam_props.zNear, 1e-3f);
	ImGui::DragFloat("Far", &cam_props.zFar, clipping_plane_prop_speed, 2000);
	ImGui::SliderFloat("Move Speed", &cam_props.moveSpeed, 0.1f, 100.0f);
	ImGui::SliderFloat("Rot Speed", &cam_props.rotSpeed, 0.1f, 20.0f);

	if (ImGui::TreeNode("Camera Prop Setting"))
	{
		ImGui::SliderFloat("Clipping Planes Near/Far edit speed", &clipping_plane_prop_speed, 0.0f, 5.0f);
		ImGui::TreePop();
	}
	if (ImGui::TreeNode("Camera State"))
	{
		ImGui::PushID(&mCamera);
		const auto& cam_state = mCamera.GetState();
		ImGui::Text("Position: %s", cam_state.position.ToString().c_str());
		ImGui::Text("Forward: %s", cam_state.forward.ToString().c_str());
		ImGui::Text("Up: %s", cam_state.up.ToString().c_str());
		ImGui::Text("Right: %s", mCamera.GetRight().ToString().c_str());
		ImGui::Text("Orientation: %s", cam_state.orientation.ToString().c_str());
		ImGui::Text("Pitch: %f", cam_state.pitch);
		ImGui::PopID();
		ImGui::TreePop();
	}
}

void Application::DrawUI_LightingPanel()
{
	vx::Vec3& light_dir = mRenderer.GetDirectionalLightDir();
	ImGui::DragFloat3("light direction", &light_dir[0], 0.1f, -1.0f, 1.0f);
	vx::Vec4 light_col = mRenderer.GetDirectionalLightColour();
	if (ImGui::ColorEdit3("light colour", &light_col[0]))
		mRenderer.SetDirectionalLightColour(vx::Colour(light_col));
	float _v = mRenderer.GetDirectionalLightIntensity();
	if (ImGui::SliderFloat("Light intesity", &_v, 0.0f, 1.0f))
		mRenderer.SetDirectionalLightIntensity(_v);

	//static vx::Quat light_rotation = vx::Quat::LookRotation(light_dir, vx::Vec3::Up());
	static vx::Vec3 angles = vx::RadToDeg(vx::Quat::LookRotation(light_dir, vx::Vec3::Up()).GetEulerAngles());
	bool changed = ImGui::DragFloat("Pitch", &angles[0], 1.0f);
	changed |= ImGui::DragFloat("Yaw", &angles[1], 1.0f);
	ImGui::Text("Roll: %d", angles.Z());


	if (changed)
	{
		angles.SetZ(0.0f);
		angles[1] = fmod(angles[1] + 180.0f, 360.0f) - 180.0f;
		angles[0] = fmod(angles[0] + 180.0f, 360.0f) - 180.0f;
		vx::Quat q = vx::Quat::FromEulerAngle(vx::DegToRad(angles));
		//from world rotation
		light_dir = (q.InverseRotate(vx::Vec3::Forward())).Normalised();
	}


	if (ImGui::TreeNodeEx("Light Quaternion"))
	{

		static vx::Vec2 drag_pad_size = vx::Vec2(200.0f);
		if (ImGui::BeginChild("Euler", ImVec2(ImGui::GetWindowWidth(), drag_pad_size.Y() + 75.0f)))
		{
			bool b = true;
			if (EditorImGui::EditEulerWithDrag(angles, b, drag_pad_size, mRenderer.GetmDirLightDebugRTPtr()->GetAttachment(0)->GetBuffer()))
			{
				//from world rotation
				vx::Quat q = vx::Quat::FromEulerAngle(vx::DegToRad(angles)).Normalised();
				light_dir = (q.InverseRotate(vx::Vec3::Forward())).Normalised();
			}
		}
		ImGui::EndChild();

		if (ImGui::TreeNode("Light Gizmos setting"))
		{
			mLightDebugProps.dirty |= ImGui::DragFloat3("view position", &mLightDebugProps.viewPosition[0], 0.1f);
			mLightDebugProps.dirty |= ImGui::DragFloat3("view target", &mLightDebugProps.viewTarget[0], 0.1f);
			mLightDebugProps.dirty |= ImGui::SliderFloat("arrow width", &mLightDebugProps.arrowWidth, 0.0f, 1.0f);
			mLightDebugProps.dirty |= ImGui::SliderFloat("arrow height", &mLightDebugProps.arrowHeight, 0.0f, 1.0f);
			ImGui::Checkbox("Enable Draw", &mLightDebugProps.enableDraw);
			ImGui::TreePop();
		}

		ImGui::TreePop();
	}

	if (ImGui::Button("Invert Light"))
	{
		light_dir = -light_dir;
		angles = vx::RadToDeg(vx::Quat::LookRotation(light_dir, vx::Vec3::Up()).GetEulerAngles());
	}

	if (ImGui::Button("Align to Camera"))
	{
		light_dir = mCamera.GetForward().Normalised();
		angles = vx::RadToDeg(vx::Quat::LookRotation(light_dir, vx::Vec3::Up()).GetEulerAngles());
	}

}

void Application::DrawUI_ShadowPanel()
{
	auto* p_shadow_data = mRenderer.ShadowDataPtr();
	ImGui::DragFloat("View Near offset", &p_shadow_data->zNearOffset, 0.005f);
	ImGui::DragFloat("View Offset", &p_shadow_data->distance, 0.05f);
	ImGui::SliderFloat("View frustum split depth", &p_shadow_data->split_depth, 0.01f, 1.0f);
	if (!p_shadow_data->HasOriginQuery())
		ImGui::DragFloat3("View Origin", &p_shadow_data->origin[0], 0.1f);
	else
		ImGui::TextColored(ImVec4(0.1f, 0.8f, 0.2, 1.0f), "Dynamic Origin, try modifing offset instead");
}



vx::RefConst<vx::Shape> Application::TryGetCreatedShape(const vx::Float3& he, float density, const vx::EShapeType shape_type) const
{
	auto CompareProp = [](const vx::RefConst<vx::Shape>& shape, const vx::Float3& he, float p, const vx::EShapeType shape_type)
		{
			if (!shape) return false;

			vx::Float3 halfExtents;
			shape->GetHalfExtents().Store(halfExtents);
			float density = shape->GetDensity();
			vx::EShapeType shapeType = shape->GetType();


			return halfExtents == he &&
				density == p &&
				shapeType == shape_type;
		};

	for (const auto& c : mCreatedShape)
		if (CompareProp(c, he, density, shape_type))
			return c;
	return nullptr;
}

void Application::PhysicsInteraction()
{
	if (!mPhysicsWorld && !mParticleWorld) return;

  	bool create_obj = Input::GetKeyDown(IKeyCode::Space) || (mNewPhyObjectSettings.allowKeyHeld && Input::GetKey(IKeyCode::Space));
	
	if (create_obj)
	{
		vx::Vec3 dir = (mNewPhyObjectSettings.spawnFromView) ? mCamera.GetForward() : (mTestCanon.spawn - mTestCanon.back).Normalised();

		/// for convenie offset in the forward direction when spawning from view 
		const Vec3 position = (mNewPhyObjectSettings.spawnFromView) ? mCamera.GetPosition() + dir * mNewPhyObjectSettings.offsetFromView : mTestCanon.back;
		const Vec3 impluse = dir * ((mNewPhyObjectSettings.applyImpulse) ? mNewPhyObjectSettings.impulse : 0.0f);

		if (mPhysicsWorld != nullptr && mNewPhyObjectSettings.type == CreatePhysicsObjectSettings::EType::Body)
		{
			vx::BodySettings& body_settings = (mNewPhyObjectSettings.isDynamic) ?
				vx::BodySettings::DefaultDynamicConstruct() : vx::BodySettings::DefaultStaticConstruct();
			body_settings.position = position;
			body_settings.density = mNewPhyObjectSettings.density;

			if(mNewPhyObjectSettings.overrideMasses)
			{
				body_settings.overrideMasses = mNewPhyObjectSettings.overrideMasses;
				body_settings.mass = mNewPhyObjectSettings.mass;
				if(mNewPhyObjectSettings.multiplyInertiaTensor_Mass)
				{
					Float3 v = mNewPhyObjectSettings.inertia;
					float m = mNewPhyObjectSettings.mass;
					body_settings.inertia = vx::Float3(v.x * m, v.y * m, v.z * m);
				}
				else
					body_settings.inertia = mNewPhyObjectSettings.inertia;
			}
			body_settings.impluse = impluse;
			body_settings.intialVelocity = dir * mNewPhyObjectSettings.initialLinearVelocity;
			body_settings.linearDamping = mNewPhyObjectSettings.damping;
			body_settings.angularDamping = mNewPhyObjectSettings.angularDamping;
			body_settings.friction = mNewPhyObjectSettings.friction;
			body_settings.restitution = mNewPhyObjectSettings.restitution;

			

			//CacheData out_cache_data;
			vx::RefConst<Shape> shape = TryGetCreatedShape(mNewPhyObjectSettings.halfExtents,
				mNewPhyObjectSettings.density, mNewPhyObjectSettings.bodyShape);


			if (shape)
				body_settings.shape = shape;
			else
			{

				float density = 0.0f;
				if (body_settings.motionType == EMotionType::Dynamic)
					//for old deprecated
					if (body_settings.mass > 0.0f) //to support deprecated method
						density = body_settings.density;

				if (mNewPhyObjectSettings.bodyShape == vx::EShapeType::Box)
				{
					vx::BoxShapeSettings settings(vx::Vec3::LoadFloat3Raw(mNewPhyObjectSettings.halfExtents));

					//for old deprecated
					//if (body_settings.mass > 0.0f) //to support deprecated method
						settings.SetDensity(body_settings.density);
					body_settings.shape = vx::MakeRef<BoxShape>(settings);
				}
				else if (mNewPhyObjectSettings.bodyShape == vx::EShapeType::Capsule)
				{
					auto& he = mNewPhyObjectSettings.halfExtents;
					float actual_half_height = he.y * 0.5f;
					vx::CapsuleShapeSettings settings(he.x, actual_half_height);

						settings.SetDensity(body_settings.density);
					body_settings.shape = vx::MakeRef<CapsuleShape>(settings);
				}
				else
				{
					vx::SphereShapeSettings settings(mNewPhyObjectSettings.halfExtents.x);
					settings.SetDensity(body_settings.density);
					body_settings.shape = vx::MakeRef<SphereShape>(settings);
				}


				//cache shape prop
				mCreatedShape.push_back(body_settings.shape);
			}
			
			body_settings.debug_name = body_settings.shape->GetShapeTypeName();

			for(int i = 0; i < mNewPhyObjectSettings.count; ++i)
				mPhysicsWorld->CreateBody(body_settings);
		}
		else if(mParticleWorld)
		{
			float mass = (mNewPhyObjectSettings.newAsAnchor) ? 0.0f : mNewPhyObjectSettings.mass;
			vx::Particles::Particle* p = mParticleWorld->CreateParticle(mTestCanon.back,
				mass, 1.0f, mNewPhyObjectSettings.damping);
			p->AddImpluse(impluse);

			if (mNewPhyObjectSettings.newAsAnchor)
				mLastAnchor = p;


			if (mNewPhyObjectSettings.attachWithLast && mParticleWorld->GetParticleCount() > 2)
			{
				auto& particles = mParticleWorld->GetParticles();

				auto last_particle = &particles[particles.size() - 2];
				mParticleWorld->CreateSpringConstraint(last_particle, p);
			}

		}
	}


	if (mPhysicsWorld && mDebugAngularImpulse.apply)
	{
		auto& bodies = mPhysicsWorld->GetBodies();
		Vec3 pointA = vx::Vec3::LoadFloat3Raw(mDebugAngularImpulse.pointA);
		Vec3 impluse =vx::Vec3::LoadFloat3Raw(mDebugAngularImpulse.impluse);
		for (auto& body : bodies)
		{
			if(mDebugAngularImpulse.onlyAngularImpluse)
				body.ApplyAngularImpulse(impluse, pointA);
			else
				body.ApplyImpulse(impluse, pointA);
		}

		mDebugAngularImpulse.apply = false;
	}



	if (mExternalEffectDynamicBodyInfo.pending)
	{
		for(int i = 0; i < mExternalEffectDynamicBodyInfo.N; ++i)
		{
			vx::Body& body = mPhysicsWorld->GetBodyManager().GetBody(mExternalEffectDynamicBodyInfo.selectedBodies[i]);

			vx::Vec3 im = vx::Vec3::LoadFloat3Raw(mExternalEffectDynamicBodyInfo.impluse);
			vx::Vec3 f = vx::Vec3::LoadFloat3Raw(mExternalEffectDynamicBodyInfo.force);
			body.AddForce(f);
			body.ApplyImpulse(im);
		}
		mExternalEffectDynamicBodyInfo.Reset();
	}

}

void Application::DrawHelpWindow(bool& p_open)
{
	if (ImGui::Begin("Help", &p_open))
	{
		ImGui::SeparatorText("Inputs & Shortcuts");
		ImGui::Text("WASD basic camera navigation");
		ImGui::Text("Q to move camera up");
		ImGui::Text("E to move camera down");
		ImGui::Text("Ctrl + N: Open Create New Physics Body window");
		ImGui::Text("Ctrl + C: Open Apply External Effect on Dynamic Body window");
		ImGui::Text("Space Bar to spawn physics body based on setting");
		ImGui::Text("R button down to Reload physics world");
		ImGui::Text("P or Pause button to Pause/Resume simulation");
		ImGui::Text("F10 button to Single Step simulation WIP");
		ImGui::Text("Left Shift + F10 button to N Step simulation WIP");
		ImGui::Text("V to toggle V-Sync");
		ImGui::Text("Mouse Right click to toggle cursor lock");

		StackString dir_light_controls("DIR LIGHT CTRLS:\n");
		dir_light_controls << "[I]/[K]: Move Light / Backward(Z - Axis)\n";
		dir_light_controls << "[J]/[L]: Move Light/Right (X-Axis)\n";
		dir_light_controls << "[U]/[O]: Elevate Light Up/Down (Y-Axis)\n";
		ImGui::Text(dir_light_controls.Data());

		ImGui::SeparatorText("Util");
		ImGui::Text("O to increase spawn impluse by 250 Ns : NEED NEED UPDATE");
		ImGui::Text("N to decrease spawn impluse by 250 Ns : NEED NEED UPDATE");
		ImGui::Spacing();
		ImGui::Text("Left Shift + N: move spawn NEED NEED UPDATE");
		ImGui::Text("Left Arrow/Num Keypad 4: move spawn right");
		ImGui::Text("Right Arrow/Num Keypad 6: move spawn left");
		ImGui::Text("Down Arrow/Num Keypad 2: move spawn backward");
		ImGui::Text("Up Arrow/Num Keypad 8: move spawn forward");
		ImGui::Text("Num Keypad 7: move spawn up");
		ImGui::Text("Num Keypad 9: move spawn down");

		
	}
	ImGui::End();
}

#include <functional>
void Application::DrawBVHNodesOverlay(bool& open)
{
	vx::BVHBroadphase<AABB>* broad_phase = nullptr;
	{
		auto* _v = mPhysicsWorld->GetBroadphase();
		if (_v->Type() != vx::EBroadphaseType::BVH)
			return;
		broad_phase = static_cast<vx::BVHBroadphase<AABB>*>(_v);
	}
#pragma region NEW

	auto* root = (broad_phase->GetTreeStat()).tree->GetRoot();
	auto* tree = (broad_phase->GetTreeStat()).tree;
	if (!root) return;

	if (ImGui::Begin("BVH Tree structure", &open))
	{
		static float node_width = 20.0f;
		static float node_height = 10.0f;
		static float vertical_spacing = 45.5f;
		static float horizontal_spacing = 35.0f;
		static float root_offset_y = 0.5f;
		static int root_depth = 0;

		if (ImGui::TreeNode("Setting"))
		{
			ImGui::DragFloat("node width", &node_width, 0.1f, 10.0f, 300.0f);
			ImGui::DragFloat("node height", &node_height, 0.1f, 10.0f, 200.0f);
			ImGui::DragFloat("vertical spacing", &vertical_spacing, 0.1f, 10.0f, 300.0f);
			ImGui::DragFloat("horizontal_spacing", &horizontal_spacing, 0.1f, 10.0f, 300.0f);
			ImGui::DragFloat("root offset y", &root_offset_y, 0.1f, 0.0f, 10.0f);
			ImGui::TreePop();
		}

		ImGui::Text("Trree Depth %d", root_depth);
		ImGui::BeginChild("BVHScroll", ImVec2(0, 0), true, ImGuiWindowFlags_HorizontalScrollbar | ImGuiWindowFlags_AlwaysVerticalScrollbar);


		std::function<int(const vx::BVHTree<AABB>::NodeID&)> tree_depth =
			[&](auto& id) -> int
			{
				if (id == vx::BVHTree<AABB>::kInvalidNode)return 0;
				const vx::BVHTree<AABB>::Node* n = tree->GetNode(id);
				if (n->IsLeaf()) return 1;
				return 1 + std::max(tree_depth(n->children[0]), tree_depth(n->children[1]));
			};

		std::function<int(const vx::BVHTree<AABB>::NodeID& id)> leaves_count =
			[&](auto& id) -> int
			{
				if (id == vx::BVHTree<AABB>::kInvalidNode)return 0;
				const vx::BVHTree<AABB>::Node* n = tree->GetNode(id);
				if (n->IsLeaf()) return 1;
				return leaves_count(n->children[0]) + leaves_count(n->children[1]);
			};


		std::unordered_map<int, int> level_counts;
		std::function<void(const vx::BVHTree<AABB>::Node*, int)> count_levels =
			[&](auto* n, int depth)
			{
				if (!n)return;
				level_counts[depth]++;
				if(n->IsInternal())
				{
					count_levels(tree->GetNode(n->children[0]), depth + 1);
					count_levels(tree->GetNode(n->children[1]), depth + 1);
				}
			};

			count_levels(root, 0);
		int max_width = std::max_element(level_counts.begin(), level_counts.end(),
			[](auto& a, auto& b) {
			return a.second < b.second;
		}) -> second;

		//{
			int depth = tree_depth(root->id);
			int leaves = leaves_count(root->id);
			root_depth = leaves;
			float content_width = leaves * (node_width + horizontal_spacing);
			float content_height = depth * (node_height + vertical_spacing);
			//ImGui::Button("test button", ImVec2(content_width, content_height));
			static float x_min = FLT_MAX;// (ImGui::GetWindowSize().x * 0.5f) - (content_width * 0.5f);
			static float y_min = FLT_MAX;// ImGui::GetCursorStartPos().y;
			//use last frame hack
			const ImVec2 local = ImVec2(x_min, y_min);
			//ImGui::SetCursorScreenPos(local);
			//ImGui::Dummy(ImVec2(content_width, content_height));

			ImGui::SetCursorPos(local);
			ImGui::Dummy(ImVec2(content_width, content_height));
			x_min = FLT_MAX;
			//y_min = FLT_MAX;
		//}

		ImDrawList* draw_list = ImGui::GetWindowDrawList();
		const ImVec2 win_pos = ImGui::GetWindowPos();
		const ImVec2 cursor_pos = ImGui::GetCursorStartPos();
		const ImVec2 offset = ImVec2(win_pos.x + cursor_pos.x, win_pos.y + cursor_pos.y);
		const ImVec2 scroll = ImVec2(ImGui::GetScrollX(), ImGui::GetScrollY());
		
		ImVec2 hack_min = ImVec2((win_pos.x + ImGui::GetWindowSize().x * 0.5f) - (content_width * 0.5f), root_offset_y + offset.y);
		//ImGui::SetCursorPos(hack_min);
		//ImGui::Button("ex button", ImVec2(content_width, content_height));

		const ImVec2 node_size(node_width, node_height);
		//Draw node box
		std::function<void(const vx::BVHTree<AABB>::NodeID&, float, int)> draw_node =
			[&](const vx::BVHTree<AABB>::NodeID id, float x, int depth) -> void
			{
				if (id == vx::BVHTree<AABB>::kInvalidNode) return;
				const vx::BVHTree<AABB>::Node* n = tree->GetNode(id);

				//Draw node text
				//ImVec2 pos = ImVec2(win_pos.x + x, win_pos.y + depth * node_spacing_y);
				//ImVec2 pos = ImVec2(offset.x + x, offset.y + depth * (node_height + vertical_spacing));
				ImVec2 pos = ImVec2(offset.x + x - scroll.x, offset.y + depth * (node_height + vertical_spacing) - scroll.y);
				ImU32 col = ImColor(n->IsLeaf() ? ImVec4(0.3f, 0.9f, 0.3f, 1.0f) : ImVec4(0.4f, 0.6f, 1.0f, 1.0f));

				if (n->parent == vx::BVHTree<AABB>::kInvalidNode)
					col = ImColor(ImVec4(0.9f, 0.1f, 0.3f, 1.0f));

				//draw box 
				ImVec2 rect_min = pos;
				ImVec2 rect_max = ImVec2(pos.x + node_size.x, pos.y + node_size.y);
				draw_list->AddRectFilled(rect_min, rect_max, col, 4.0f);
				draw_list->AddRect(rect_min, rect_max, IM_COL32_WHITE, 4.0f);
				int p = (n->parent == vx::BVHTree<AABB>::kInvalidNode) ? -1 : n->parent;
				int c0 = (n->children[0] == vx::BVHTree<AABB>::kInvalidNode) ? -1 : n->children[0];
				int c1 = (n->children[1] == vx::BVHTree<AABB>::kInvalidNode) ? -1 : n->children[1];
				draw_list->AddText(ImVec2(pos.x + 5, pos.y + 5), IM_COL32_WHITE, ("D: " + std::to_string(depth) +
					"\nID: " + std::to_string(id) +
					",P: " + std::to_string(p) +
					"\nC0: " + std::to_string(c0) + 
					",C1: " + std::to_string(c1) + "\nhas body" + std::to_string(n->body != nullptr)).c_str());
				//get min in window space not screen i.e window_pos - rect_min
				//and consider the x offset 
				x_min = std::min(x_min, rect_min.x - win_pos.x);
				y_min = std::min(y_min, rect_min.y - win_pos.y);

				//draw children 
				if (n->IsInternal())
				{
					int left_count = leaves_count(n->children[0]);
					int right_count = leaves_count(n->children[1]);
					//float total_width = static_cast<float>(left_count + right_count) * node_spacing_x * 0.5f;
					float total_width = static_cast<float>(left_count + right_count) * (node_width + horizontal_spacing);

					ImVec2 parent_center = ImVec2(pos.x + node_size.x * 0.5f, pos.y + node_size.y);

					if (n->children[0])
					{
						//ImVec2 child_pos = ImVec2(x - total_width * 0.5f, (depth + 1) * node_spacing_y);
						//ImVec2 child_center = ImVec2(offset.x + child_pos.x + node_size.x * 0.5f, 
						//	offset.y + child_pos.y);

						//draw_list->AddLine(parent_center, child_center, IM_COL32(200, 200, 200, 255));
						//draw_node(n->children[0], x - total_width * 0.5f, depth + 1);

						float child_x = x - total_width * 0.25f;
						ImVec2 child_center = ImVec2(offset.x + child_x + node_size.x * 0.5f - scroll.x,
										offset.y + (depth + 1) * (node_height + vertical_spacing) - scroll.y);

						draw_list->AddLine(parent_center, child_center, IM_COL32(200, 200, 200, 255));
						draw_node(n->children[0], child_x, depth + 1);
					}

					if (n->children[1])
					{
						//ImVec2 child_pos = ImVec2(x + total_width * 0.5f, (depth + 1) * node_spacing_y);
						//ImVec2 child_center = ImVec2(offset.x + child_pos.x + node_size.x * 0.5f,
						//	offset.y + child_pos.y);

						//draw_list->AddLine(parent_center, child_center, IM_COL32(200, 200, 200, 255));
						//draw_node(n->children[1], x + total_width * 0.5f, depth + 1);

						float child_x = x + total_width * 0.25f;
						ImVec2 child_center = ImVec2(offset.x + child_x + node_size.x * 0.5f - scroll.x,
							offset.y + (depth + 1) * (node_height + vertical_spacing) - scroll.y);

						draw_list->AddLine(parent_center, child_center, IM_COL32(200, 200, 200, 255));
						draw_node(n->children[1], child_x, depth + 1);
					}
				}
			};

		//from draw from root
		//float root_offset_x = content_width * 0.5f;
		float root_offset_x = ImGui::GetWindowSize().x * 0.5f;
		draw_node(root->id, root_offset_x, root_offset_y);
		ImGui::EndChild();
	}
	ImGui::End();
	
#pragma endregion
}




void Application::DrawApplyExternalEffectDynamicBody(bool* p_open)
{
	if (!ImGui::Begin("Apply External Effect on Dynamic Body", p_open))
	{
		ImGui::End();
		return;
	}

	ImGui::SeparatorText("App Camera");

	ImGui::DragFloat3("Location", &mCameraConfig.teleportLocation[0], 0.1f);
	ImGui::DragFloat3("Look At", &mCameraConfig.teleportLookAt[0], 0.1f);
	ImGui::Checkbox("Keep View dir", &mCameraConfig.keepViewDir);
	if (ImGui::Button("Teleport"))
	{
		vx::Vec3 pos = vx::Vec3::LoadFloat3Raw(mCameraConfig.teleportLocation);
		vx::Vec3 dir = (vx::Vec3::LoadFloat3Raw(mCameraConfig.teleportLookAt) - pos).Normalise();
		vx::Vec3 up = vx::Vec3::Up();
		vx::Vec3 rt = vx::Vec3::Cross(dir, up).Normalised();
		up = vx::Vec3::Cross(rt, dir);
		Camera::State cam_state = Camera::State(pos, dir, up);
	//	cam_state.pitch = cam_state.orientation.GetEulerAngles().X();
		mCamera.SetState(cam_state);
	}

	ImGui::SeparatorText("External Effect on Dynamic Body");

	
	if (!mPhysicsWorld)
	{
		ImGui::End();
		return;
	}
	/// what to be apply to do 
	/// Apply impluse, force 
	/// 
	/// able to teleport to view location; 
	/// 
	/// select rigidbody 
	/// types 
	/// first dynamic body 
	/// last dynamic body
	/// N dynamic body
	/// 
	/// 
	enum class ESelectType : uint8
	{
		First,
		Last,
	};

	constexpr const char* Select_Bodies_Type_Names =
		"First Dynamic Body\0"
		"Last Dynamic Body\0"
		"\0";
	static ESelectType type_select;

	static bool new_select = true;
	new_select |= EditorImGui::Combo("Selection Config", type_select, Select_Bodies_Type_Names);
	
	static int n_bodies = 1;
	static int last_n_bodies = 1;
	new_select |= ImGui::SliderInt("N - Bodies", &n_bodies, 1, ExternalEffectDynamicBodyInfo::kMaxBodies);
	new_select |= ImGui::SliderInt("Last N - Bodies", &last_n_bodies, 1, ExternalEffectDynamicBodyInfo::kMaxBodies);

	if (new_select)
	{
		mExternalEffectDynamicBodyInfo.ClearSelect();
		int i = (type_select == ESelectType::First) ? 0 : int(mPhysicsWorld->GetBodies().size() - last_n_bodies);
		vx::BodyID first_selected_body;

		bool found_first = false;
		//for (;i < mPhysicsWorld->GetBodies().size();++i)
		for (int steps = 0; steps < mPhysicsWorld->GetBodies().size(); i = (i + 1) % mPhysicsWorld->GetBodies().size(), ++steps)
		{
			if (mPhysicsWorld->GetBodies()[i].IsDynamic())
			{
				auto body = mPhysicsWorld->GetBodies()[i];
				///cpu should be able to predicted this, as it done once 
				if (!found_first)
				{
					first_selected_body = body.GetID();
					found_first = true;
				}
				//this check is for if this is the first it meant we have looped around
				else if (first_selected_body == body.GetID()) break;
				//mExternalEffectDynamicBodyInfo.Add(mPhysicsWorld->GetBodyManager().GetBodyDebugInfo(body));
				mExternalEffectDynamicBodyInfo.AddUnsafe(body.GetID());
			}
			if (mExternalEffectDynamicBodyInfo.GetInstanceCount() >= n_bodies || mExternalEffectDynamicBodyInfo.N >= ExternalEffectDynamicBodyInfo::kMaxBodies) break;
		}
	}
	if (ImGui::TreeNode("Debug Target Dynamic Bodies"))
	{
		for (int j = 0; j < mExternalEffectDynamicBodyInfo.GetInstanceCount(); ++j)
		{
			//ImGui::Selectable(mExternalEffectDynamicBodyInfo.selectedBodies[j].name.c_str());
			ImGui::Selectable(mPhysicsWorld->GetBodyManager().GetBodyDebugName(mExternalEffectDynamicBodyInfo.selectedBodies[j]));
		}
		ImGui::TreePop();

	}

	enum class EForceImpluseMode : vx::uint8
	{
		Directional,
		Camera_Axes,
		World_Axes,
	};

	constexpr const char* ApplyForceImpluseMode = 
		"Directional\0"
		"Camera Axes\0"
		"World Axes\0""\0";

	enum class EAxis : vx::uint8
	{
		Right,
		Up,
		Forward
	};

	constexpr const char* Axis_Mode_Names =
		"Right\0"
		"Up\0"
		"Forward\0""\0";

	static EForceImpluseMode force_impluse_mode;
	static EAxis axis = EAxis::Right;

	int enum_v = int(force_impluse_mode);
	EditorImGui::Combo("Apply Force Impluse Mode", force_impluse_mode, ApplyForceImpluseMode);
	if (force_impluse_mode != EForceImpluseMode::Directional)
		EditorImGui::Combo("Axis Mode Names", axis, Axis_Mode_Names);

	using Drag_Value_Edit = bool(*)(const char*, float*, float, float, float, const char*, int);

	static const Drag_Value_Edit drag_value_edit[2] =
	{
		&ImGui::DragFloat3,
		&ImGui::DragFloat
	};




	///build matrix
	const vx::Vec3 direction_axes[4][3] =
	{
		/// Directional,
		{
			vx::Vec3(1.0f),
			vx::Vec3(1.0f),
			vx::Vec3(1.0f),
		},
		/// Camera_Axes,
		{
			mCamera.GetRight(),
			mCamera.GetUp(),
			mCamera.GetForward(),
		},
		/// World_Axes,
		{
			vx::Vec3::Right(),
			vx::Vec3::Up(),
			vx::Vec3::Forward(),
		},
		/// Local_Axes
		//hack for now 
		{
			vx::Vec3::Right(),
			vx::Vec3::Up(),
			vx::Vec3::Forward(),
		},
	};

	
	static vx::Vec3 impluse(1.0f);
	static vx::Vec3 force(1.0f);
	int type3 = int(force_impluse_mode != EForceImpluseMode::Directional);
	drag_value_edit[type3]("Impulse", &impluse[0], 0.1f, (0.0f), (0.0f), "%.3f", 0);
	ImGui::SameLine(); 
	if (ImGui::Button("Apply Impluse"))
	{
		vx::Vec3 dir = direction_axes[int(force_impluse_mode)][int(axis)];

		if (force_impluse_mode != EForceImpluseMode::Directional)
			impluse = impluse.SplatX();
		(dir * impluse).Store(mExternalEffectDynamicBodyInfo.impluse);
		mExternalEffectDynamicBodyInfo.pending = true;
	}
	drag_value_edit[type3]("Force", &force[0], 0.1f, (0.0f), (0.0f), "%.3f", 0);
	ImGui::SameLine();
	if (ImGui::Button("Apply Force"))
	{
		vx::Vec3 dir = direction_axes[int(force_impluse_mode)][int(axis)];

		if (force_impluse_mode != EForceImpluseMode::Directional)
			force = force.SplatX();
		(dir * force).Store(mExternalEffectDynamicBodyInfo.force);
		mExternalEffectDynamicBodyInfo.pending = true;
	}
	if (ImGui::Button("Apply Impluse & Force"))
	{
		vx::Vec3 dir = direction_axes[int(force_impluse_mode)][int(axis)];

		if (force_impluse_mode != EForceImpluseMode::Directional)
		{
			impluse = impluse.SplatX();
			force = force.SplatX();
		}
		(dir * impluse).Store(mExternalEffectDynamicBodyInfo.impluse);
		(dir * force).Store(mExternalEffectDynamicBodyInfo.force);
		mExternalEffectDynamicBodyInfo.pending = true;
	}

	if (ImGui::TreeNode("Debug"))
	{
		ImGui::Text("Direction Axis: %s", direction_axes[int(force_impluse_mode)][int(axis)].ToString().c_str());

		
		ImGui::TreePop();
	}
	
	ImGui::End(); //<-- if (!ImGui::Begin("Apply External Effect on Dynamic Body", p_open))
}

void Application::CreateNewPhysicsBodyWindow()
{

	if(ImGui::Begin("Create New Physics Body", &mNewPhyObjectSettings.openWindow))
	{
		constexpr const char* type_names[] = { "Body", "Particle" };
		int curr_type = static_cast<int>(mNewPhyObjectSettings.type);
		if (ImGui::Combo("Type", &curr_type, type_names, IM_ARRAYSIZE(type_names)))
			mNewPhyObjectSettings.type = static_cast<CreatePhysicsObjectSettings::EType>(curr_type);

		ImGui::Checkbox("Allow key held", &mNewPhyObjectSettings.allowKeyHeld);

		//ImGui::SeparatorText("New Physics Body");
		if (mNewPhyObjectSettings.type == CreatePhysicsObjectSettings::EType::Body)
		{
			constexpr const char* sp_type_names[] = { "Sphere", "Box", "Capsule"};
			int curr_sp_type = static_cast<int>(mNewPhyObjectSettings.bodyShape);
			if (ImGui::Combo("Body Shape Type", &curr_sp_type, sp_type_names, IM_ARRAYSIZE(sp_type_names)))
			{
				mNewPhyObjectSettings.bodyShape = static_cast<vx::EShapeType>(curr_sp_type);
				///alway reset when shape type change to prevent bugs 
				mNewPhyObjectSettings.halfExtents = Float3{ 0.5f };
				switch (mNewPhyObjectSettings.bodyShape)
				{
				case vx::EShapeType::Sphere:
					mNewPhyObjectSettings.inertia = vx::BodySettings::UnitSphereinteriatensor();
					mNewPhyObjectSettings.halfExtents = Float3{ 0.5f };
					break;
				case vx::EShapeType::Box:
					mNewPhyObjectSettings.inertia = vx::BodySettings::UnitBoxinteriatensor();
					mNewPhyObjectSettings.halfExtents = Float3{ 0.5f };
					break;
				case vx::EShapeType::Capsule:
					mNewPhyObjectSettings.inertia = vx::BodySettings::UnitCapsuleinteriatensor();
					mNewPhyObjectSettings.halfExtents = Float3{ 0.5f, 1.0f, 0.5f };// convert to the actual half extent along y
					break;
				default:
					break;
				}
	
			}

			ImGui::SliderInt("Count", (int*)&mNewPhyObjectSettings.count, 1, 50);

			switch (mNewPhyObjectSettings.bodyShape)
			{
			case vx::EShapeType::Sphere: ImGui::DragFloat("Radius (m)", &mNewPhyObjectSettings.halfExtents[0], 0.1f, 0.0f);
				break;
			case vx::EShapeType::Box:  ImGui::DragFloat3("Half Extents", &mNewPhyObjectSettings.halfExtents[0], 0.1f, 0.0f); 
				break;
			case vx::EShapeType::Capsule: 
				ImGui::DragFloat("Radius (m)", &mNewPhyObjectSettings.halfExtents[0], 0.1f, 0.0f);
				
				ImGui::DragFloat("Half Capsule height (m)", &mNewPhyObjectSettings.halfExtents[1], 0.1f, 0.0f);
				break;
			default:
				VX_LOG_WARN("UNKNOWN Create Shape type!!!");
				ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f), "UNKNOWN Create Shape type!!!");
				break;

			}

			ImGui::Checkbox("Apply Impluse", &mNewPhyObjectSettings.applyImpulse);
			ImGui::DragFloat("Impulse (kg m/s)", &mNewPhyObjectSettings.impulse);
			ImGui::DragFloat("Initial Velocity", &mNewPhyObjectSettings.initialLinearVelocity);
			ImGui::Checkbox("Shoot from view", &mNewPhyObjectSettings.spawnFromView);
			ImGui::SliderFloat("Offset from view", &mNewPhyObjectSettings.offsetFromView, 0.0f, 20.0f);
			
			ImGui::Spacing();
			ImGui::DragFloat("Density (kg/m^3)", &mNewPhyObjectSettings.density, 0.0f);
			ImGui::Checkbox("Override Mass WIP", &mNewPhyObjectSettings.overrideMasses);
			if(mNewPhyObjectSettings.overrideMasses)
			{
				ImGui::SliderFloat("Mass (kg)", &mNewPhyObjectSettings.mass, 0.0f, 200.0f);
				ImGui::DragFloat3("Inertia Tensor ", &mNewPhyObjectSettings.inertia[0]);
				ImGui::Checkbox("Multiply Inertia Tensor with Mass, for final Inertia", &mNewPhyObjectSettings.multiplyInertiaTensor_Mass);
			}
			ImGui::SliderFloat("Linear Damping", &mNewPhyObjectSettings.damping, 0.0f, 1.0f);
			ImGui::SliderFloat("Angular Damping", &mNewPhyObjectSettings.angularDamping, 0.0f, 1.0f);

			ImGui::Spacing();
			ImGui::SliderFloat("Friction Coeff", &mNewPhyObjectSettings.friction, 0.0f, 1.0f);
			ImGui::SliderFloat("Restitution Coeff", &mNewPhyObjectSettings.restitution, 0.0f, 1.0f);
			ImGui::Checkbox("Dynamic", &mNewPhyObjectSettings.isDynamic);
		}

		if (mNewPhyObjectSettings.type == CreatePhysicsObjectSettings::EType::Particle)
		{
			ImGui::Checkbox("Attach With Last Particle", &mNewPhyObjectSettings.attachWithLast);
			ImGui::Separator();
			ImGui::SeparatorText("Last Particle Anchor");
			ImGui::Checkbox("Make New Particle an Anchor", &mNewPhyObjectSettings.newAsAnchor);
			if (mLastAnchor)
			{
				vx::Vec3 pos = mLastAnchor->mPosition;
				if (ImGui::DragFloat3("mPosition: %s", &pos[0], 0.01f))
					mLastAnchor->mPosition = pos;
			}
		}
	}
	ImGui::End();
}

void Application::PhysicsCollisionWindows()
{

}


static void bind_texture_sampler(const ImDrawList* parent_list, const ImDrawCmd* cmd)
{
	struct SamplerHandle
	{
		uint32_t sampler;
	};

	SamplerHandle* handle = (SamplerHandle*)cmd->UserCallbackData;
	if (!handle)return;
	glBindSampler(0, handle->sampler);
}

void Application::DrawUIRendererResourcesPanel(/*bool* p_open*/)
{
	//if(ImGui::Begin("Renderer Resources", p_open))
	//{
		ImVec2 win_size = ImGui::GetWindowSize();
		//float target_img_width = win_size.x * 0.75f;
		static ImVec2 preview_img_size = ImVec2(200, 200);
		static bool unified_size = true;
		ImGui::Text("Image Size");
		ImVec2 size = ImGui::GetItemRectSize();


		ImGui::SameLine();
		if (ImGui::DragFloat("X", &preview_img_size[0]) && unified_size)
			preview_img_size.y = preview_img_size.x;
		ImGui::InvisibleButton("##", size);
		ImGui::SameLine();
		if (ImGui::DragFloat("Y", &preview_img_size[1]) && unified_size)
			preview_img_size.x = preview_img_size.y;
		ImGui::SameLine();  ImGui::Checkbox("Unified Size", &unified_size);


		static vx::Vec2 img_uv0 = vx::Vec2(0.0f, 1.0f);
		static vx::Vec2 img_uv1 = vx::Vec2(1.0f, 0.0f);
		ImGui::DragFloat2("Image UV0", &img_uv0[0], 0.1f);
		ImGui::DragFloat2("Image UV1", &img_uv1[0], 0.1f);

		static float scale = 1.0f;
		if (ImGui::SliderFloat("Scale", &scale, 0.0, 5.0f, "%.2f"))
			scale = VxMax(0.0f, scale);


		/////new 
		vx::Vec2 uv_center = (img_uv0 + img_uv1) * 0.5f;
		static float zoom = 1.0f;
		zoom = scale;
		zoom = VxClamp(zoom, 0.1f, 10.0f);
		float half_size = (img_uv0.X() - img_uv1.X()) * 0.5f / zoom;

		static ImVec4 img_border_col = ImVec4(1, 1, 1, 1);

		struct SamplerHandle
		{
			uint32_t sampler;
		};

		static SamplerHandle handle{ mRenderer.GetASampler()->ID() };

		ImDrawList* dl = ImGui::GetWindowDrawList();
		dl->AddCallback(bind_texture_sampler, &handle);


		std::function<void(const Texture* tex, vx::uint32 item_table_no)> draw_texture =
			[&](const Texture* tex, vx::uint32 item_table_no)
			{
				//ImVec2 _img_uv0 = ImVec2(img_uv0.X() * scale, img_uv0.Y() * scale);
				//ImVec2 _img_uv1 = ImVec2(img_uv1.X() * scale, img_uv1.Y() * scale);

				ImVec2 _img_uv0 = ImVec2(uv_center.X() - half_size, uv_center.Y() - half_size);
				ImVec2 _img_uv1 = ImVec2(uv_center.X() + half_size, uv_center.Y() + half_size);

				ImVec2 _img_size = preview_img_size;
				_img_size.y *= static_cast<float>(tex->Height()) / static_cast<float>(tex->Width());
				ImGui::TableSetColumnIndex(0);
				ImGui::Text("%d", item_table_no);
				ImGui::TableSetColumnIndex(1);
				ImGui::Text("%s, GPU ID: %d", tex->GetDebugName().data(), tex->ID());
				ImGui::SameLine();
				ImGui::TableSetColumnIndex(2);
				ImGui::Image((ImTextureID)(intptr_t)tex->ID(), _img_size,
					_img_uv0, _img_uv1,
					ImVec4(1, 1, 1, 1), img_border_col);
			};

		ImGuiTableFlags imgui_flags = ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_Resizable;
		ImGui::BeginTable("table test", 3, imgui_flags);

		float img_padding = 1.0f;
		ImGui::TableSetupColumn("##", ImGuiTableColumnFlags_WidthFixed);
		ImGui::TableSetupColumn("Name", ImGuiTableColumnFlags_WidthStretch);
		ImGui::TableSetupColumn("Texture", ImGuiTableColumnFlags_WidthStretch);
		ImGui::TableHeadersRow();




		//std::vector<int> test = { 42, 2424, 4241, 43124 };
		//for (auto it = test.begin(); it != test.end(); ++it)
		//{
		//	std::cout << "value it: " << *it << std::endl;
		//}


		int table_no = 1;



		//best way tabulate

		//Texture* p_tex = mRenderer.GetTextures();
		//ImGui::TableNextRow();
		//draw_texture(p_tex->GetDebugName().data(), p_tex->ID(), table_no++);
		//ImGui::TableNextRow();
		//draw_texture(p_tex->GetDebugName().data(), p_tex->ID(), table_no++);

		auto& textures = mRenderer.GetTextures();
		for (auto it = textures.begin(); it != textures.end(); ++it)
		{
			ImGui::TableNextRow();
			draw_texture(*it, table_no++);
		}

		//ImGui::Separator();
		ImGui::EndTable();
	//}
	//ImGui::End();


}
