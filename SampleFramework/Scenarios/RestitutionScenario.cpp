#include "SimpleBasicScenario.h"
#include "PhysicsWorld.h"

#include "Vortrix/Maths/ScalarMath.h"

#include "Collision/Shapes/Shape.h"
#include "Collision/Shapes/SphereShape.h"
#include "Collision/Shapes/BoxShape.h"
#include "Collision/Shapes/PlaneShape.h"
#include "Collision/Shapes/CapsuleShape.h"
#include "RestitutionScenario.h"

#include "Camera.h"



void RestitutionScenario::Init(vx::PhysicsWorld* i_world)
{
	Scenario::Init(i_world);

	/// Ground plane
	//CreateGroundPlane(100.0f);
	vx::BodySettings bodies_settings = vx::BodySettings::DefaultStaticConstruct();
	bodies_settings.debug_name = "ground";
	bodies_settings.shape = new vx::PlaneShape(vx::Vec3::Up(), 100.0f);
	bodies_settings.restitution = 1.0f;
	mPhysicsWorld->CreateBody(bodies_settings);


	//hack for now 
	auto& phys_solver_settings = mPhysicsWorld->GetSettings().solver;
	//mCacheData.restitutionCombine = phys_solver_settings.restitutionCombineMode;
	//mCacheData.velocityIterations = phys_solver_settings.velocityIterations;

	//this ensures that ground r = 1, then multiply 0.0f, 0.1, 0.2 yield 0.0, 0.1, 0.2....
	phys_solver_settings.velocityIterations = 15.0f;
	mPhysicsWorld->SetRestitutionCombineMode(vx::ECombineMode::Multiply);

	///cam 
	/// -16.92, 11.09, 10.015
	/// 
	if (mAppCamera)
	{
		vx::Vec3 pos = vx::Vec3(-21.48, 10.84f, 13.58f);
		vx::Vec3 fwd = vx::Vec3(0.76f, -0.31f, -0.57f);
		vx::Vec3 up = vx::Vec3(0.25f, 0.95f, -0.19f);
		Camera::State cam_state = Camera::State(pos, fwd, up);
		mAppCamera->SetState(cam_state);
	}

	vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();
	dyn_bodies_settings.debug_name = "shape";
	//dyn_bodies_settings.linearDamping = 0.0f;
//	dyn_bodies_settings.shape = new vx::SphereShape(0.5f);// vx::BoxShape(0.5f);//vx::SphereShape(0.5f);

	float height_above_ground = 5.0f;
	float z_lateral_offset = 4.0f;

	vx::Shape* capsule_shape = new vx::CapsuleShape(0.5f, 0.5f);
	vx::Shape* sphere_shape = new vx::SphereShape(1.0f);
	vx::Shape* box_shape = new vx::BoxShape(1.0f);

	float start_x = -15.0f;
	float x_lateral_offset = 3.0f;
	for (int i = 0; i < 10; ++i)
	{
		dyn_bodies_settings.restitution = 1.0f - (0.1f * i);

		/// capsule
		dyn_bodies_settings.debug_name = "capsule";
		dyn_bodies_settings.position = vx::Vec3(start_x + (x_lateral_offset * i), height_above_ground, -z_lateral_offset);
		dyn_bodies_settings.shape = capsule_shape;
		mPhysicsWorld->CreateBody(dyn_bodies_settings);
		
		
		/// sphere 
		dyn_bodies_settings.debug_name = "sphere";
		dyn_bodies_settings.position = vx::Vec3(start_x + (x_lateral_offset * i), height_above_ground, 0.0f);
		dyn_bodies_settings.shape = sphere_shape;
		mPhysicsWorld->CreateBody(dyn_bodies_settings);

		/// box
		dyn_bodies_settings.debug_name = "box";
		dyn_bodies_settings.position = vx::Vec3(start_x + (x_lateral_offset * i), height_above_ground, z_lateral_offset);
		dyn_bodies_settings.shape = box_shape;
		mPhysicsWorld->CreateBody(dyn_bodies_settings);
	}

	
}

RestitutionScenario::~RestitutionScenario()
{
	//if(mPhysicsWorld != nullptr)
	//{
	//	auto& phys_solver_settings = mPhysicsWorld->GetSettings().solver;
	//	
	//	if (mCacheData.restitutionCombine != phys_solver_settings.restitutionCombineMode)
	//	{

	//	}
	//	mCacheData.restitutionCombine = phys_solver_settings.restitutionCombineMode;
	//	mCacheData.velocityIterations = phys_solver_settings.velocityIterations;

	//	//this ensures that ground r = 1, then multiply 0.0f, 0.1, 0.2 yield 0.0, 0.1, 0.2....
	//	phys_solver_settings.restitutionCombineMode = vx::ECombineMode::Multiply;
	//	phys_solver_settings.velocityIterations = 10.0f;
	//	mPhysicsWorld->mSolverSettingDirty = true;
	//}
}


