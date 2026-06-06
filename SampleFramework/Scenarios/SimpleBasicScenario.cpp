#include "SimpleBasicScenario.h"
#include "PhysicsWorld.h"

#include "Vortrix/Maths/ScalarMath.h"

#include "Collision/Shapes/Shape.h"
#include "Collision/Shapes/SphereShape.h"
#include "Collision/Shapes/BoxShape.h"
#include "Collision/Shapes/PlaneShape.h"
#include "Collision/Shapes/CapsuleShape.h"

#include "Camera.h"

void SimpleBasicScenario::Init(vx::PhysicsWorld* i_world)
{
	Scenario::Init(i_world);

	//have physics context to config for scenario
	//io_world->mContactConstraintSolver.SetPhysicsContext(&io_world->mContext);

	if (mAppCamera)
	{
		vx::Vec3 pos = vx::Vec3(1.59, 11.24f, 21.51f);
		vx::Vec3 fwd = vx::Vec3(0.02f, -0.21f, -0.98f);
		vx::Vec3 up = vx::Vec3(0.00f, 0.98f, -0.21f);
		Camera::State cam_state = Camera::State(pos, fwd, up);
		mAppCamera->SetState(cam_state);
	}

	static float no_static_bodies = 0.0f;

	bool create_capsules = true;
	bool create_boxes = true;

	vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();
	vx::BodySettings static_bodies_settings = vx::BodySettings::DefaultStaticConstruct();
	if (create_boxes)
	{
		vx::Ref<BoxShape> unit_box = vx::MakeRef<BoxShape>(0.5f);
		dyn_bodies_settings.position = vx::Vec3(-2.0f, 5.5f, 0.0f);
		dyn_bodies_settings.debug_name = "box";
		dyn_bodies_settings.shape = vx::MakeRef<BoxShape>(0.5f, 0.25f, 0.5f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);

		dyn_bodies_settings.position = vx::Vec3(2.0f, 3.0f, 0.0f);
		dyn_bodies_settings.shape = vx::MakeRef<BoxShape>(0.15f, 0.6f, 0.35f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);

		dyn_bodies_settings.shape = unit_box;
		dyn_bodies_settings.position = vx::Vec3(-4.0f, 5.5f, 0.0f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);

		dyn_bodies_settings.position = vx::Vec3(-4.0f, 10.0f, 0.0f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);

		dyn_bodies_settings.position = vx::Vec3(1.0f, 7.5f, 0.0f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);

		dyn_bodies_settings.position = vx::Vec3(0.0f, 12.5f, 0.0f);
		dyn_bodies_settings.shape = vx::MakeRef<BoxShape>(1.5f, 0.15f, 0.15f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);
	}

	vx::SphereShapeSettings shape_settings(0.5f);
	shape_settings.SetDensity(0.0f);
	vx::Ref<vx::SphereShape> static_unit_sphere = vx::MakeRef<vx::SphereShape>(shape_settings);
	static_bodies_settings = vx::BodySettings::DefaultStaticConstruct();
	static_bodies_settings.position = vx::Vec3(3.0f, 5.5f, 0.0f);
	static_bodies_settings.debug_name = "sphere";
	static_bodies_settings.shape = static_unit_sphere;
	mPhysicsWorld->CreateBody(static_bodies_settings);

	mPhysicsWorld->CreateBody(static_bodies_settings);
	static_bodies_settings.position = vx::Vec3(6.0f, 5.5f, 0.1f);
	mPhysicsWorld->CreateBody(static_bodies_settings);
	static_bodies_settings.position = vx::Vec3(4.5f, 8.35f, 0.0f);
	mPhysicsWorld->CreateBody(static_bodies_settings);
	static_bodies_settings.position = vx::Vec3(4.5f, 3.3f, -0.1f);
	mPhysicsWorld->CreateBody(static_bodies_settings);


	/// Ground plane
	CreateGroundPlane(100.0f);

	/// Create side planes as well
	///front 
	/// back
	/// right
	/// left
	/// 
	/// front offset along z and face -z (rotate around -x)
	vx::BodySettings body_settings = vx::BodySettings::DefaultStaticConstruct();

	vx::PlaneShapeSettings plane_shape_settings(vx::Vec3::Up(), 100.0f);
	plane_shape_settings.SetDensity(0.0f);

	body_settings.debug_name = "front plane";
	body_settings.shape = vx::MakeRef<vx::PlaneShape>(plane_shape_settings);

	float world_size = 100.0f;
	body_settings.position = vx::Vec3(0.0f, 0.0f, world_size);
	body_settings.orientation.SetAxisAngle(vx::Vec3::Right(), vx::DegToRad(-90.0f));
	mPhysicsWorld->CreateBody(body_settings);

	/// front offset along -z and face z
	body_settings.debug_name = "back plane";
	body_settings.position = vx::Vec3(0.0f, 0.0f, -world_size);
	body_settings.orientation.SetAxisAngle(vx::Vec3::Right(), vx::DegToRad(90.0f));
	mPhysicsWorld->CreateBody(body_settings);

	/// front offset along x and face -x (rotate around z
	body_settings.debug_name = "right plane";
	body_settings.position = vx::Vec3(world_size, 0.0f, 0.0f);
	body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
	mPhysicsWorld->CreateBody(body_settings);

	/// front offset along -x and face x (rotate around z)
	body_settings.debug_name = "left plane";
	body_settings.position = vx::Vec3(-world_size, 0.0f, 0.0f);
	body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
	mPhysicsWorld->CreateBody(body_settings);

	if (create_capsules)
	{
		dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();
		dyn_bodies_settings.debug_name = "Capsule";
		dyn_bodies_settings.shape = vx::MakeRef<vx::CapsuleShape>(0.5f, 0.5f);
		dyn_bodies_settings.position = vx::Vec3(4.0f, 10.0f, 0.0f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);

		dyn_bodies_settings.position = vx::Vec3(7.5f, 10.0f, 0.0f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);

		dyn_bodies_settings.position = vx::Vec3(-4.5f, 10.0f, 0.0f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);

		dyn_bodies_settings.position = vx::Vec3(-7.5f, 12.0f, 0.0f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);

		dyn_bodies_settings.position = vx::Vec3(0.5f, 12.0f, 0.0f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);
	}



	if (create_boxes)
	{
		dyn_bodies_settings.position = vx::Vec3(-2.0f, 5.5f, 0.0f);
		dyn_bodies_settings.debug_name = "box";
		dyn_bodies_settings.shape = vx::MakeRef<BoxShape>(0.5f, 0.25f, 0.5f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);


		dyn_bodies_settings.position = vx::Vec3(1.0f, 7.0f, 0.0f);
		dyn_bodies_settings.shape = vx::MakeRef<BoxShape>(0.15f, 0.6f, 0.35f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);

		dyn_bodies_settings.position = vx::Vec3(0.0f, 5.5f, 0.0f);
		dyn_bodies_settings.shape = vx::MakeRef<BoxShape>(0.25f, 0.125, 0.25f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);


		dyn_bodies_settings.position = vx::Vec3(3.0f, 7.5f, 0.0f);
		dyn_bodies_settings.shape = vx::MakeRef<BoxShape>(1.0f, 0.25f, 0.25f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);


		dyn_bodies_settings.position = vx::Vec3(0.0f, 12.5f, 0.0f);
		dyn_bodies_settings.shape = vx::MakeRef<BoxShape>(1.5f, 0.15f, 0.15f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);

		dyn_bodies_settings.position = vx::Vec3(0.0f, 10.0f, 0.0f);
		dyn_bodies_settings.shape = vx::MakeRef<BoxShape>(1.0f, 0.25f, 1.0f);
		mPhysicsWorld->CreateBody(dyn_bodies_settings);
	}
}
