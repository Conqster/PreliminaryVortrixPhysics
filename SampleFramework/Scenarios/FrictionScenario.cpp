#include "FrictionScenario.h"
#include "PhysicsWorld.h"

#include "Collision/Shapes/Shape.h"
#include "Collision/Shapes/SphereShape.h"
#include "Collision/Shapes/BoxShape.h"
#include "Collision/Shapes/PlaneShape.h"

#include "Camera.h"

void FrictionScenario::Init(vx::PhysicsWorld* i_world)
{
	Scenario::Init(i_world);

	vx::Quat orientation = vx::Quat::FromAxisAngle(vx::Vec3::Forward(), vx::DegToRad(mSlopAngle));
	/// Ground plane
	//CreateGroundPlane(100.0f);
	float plane_half_size = 100.0f;
	vx::BodySettings bodies_settings = vx::BodySettings::DefaultStaticConstruct();
	bodies_settings.debug_name = "ground";
	bodies_settings.shape = new vx::PlaneShape(vx::Vec3::Up(), plane_half_size);
	bodies_settings.orientation = orientation;
	bodies_settings.friction = 1.0f;
	bodies_settings.restitution = 0.0f;
	mPhysicsWorld->CreateBody(bodies_settings);

	//hack for now 
	//this ensures that ground r = 1, then multiply 0.0f, 0.1, 0.2 yield 0.0, 0.1, 0.2....
	mPhysicsWorld->SetRestitutionCombineMode(vx::ECombineMode::Minimum);
	mPhysicsWorld->SetFrictionCombineMode(vx::ECombineMode::Multiply);


	vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();
	dyn_bodies_settings.debug_name = "shape";
	dyn_bodies_settings.restitution = 0.0f;

	vx::Shape* sphere_shape = new vx::SphereShape(1.0f);
	vx::Shape* box_shape = new vx::BoxShape(1.0f);
	
	float offset_center_along_plane_sur_tan = plane_half_size * 0.7f;
	vx::Vec2 shape_half_sizes = vx::Vec2(1.0f + 0.2f);//plus noise
	//float x_coord = 40.0f;/*11.0f*/;// 0.0f;
	float x_coord = offset_center_along_plane_sur_tan * vx::VxCos(vx::DegToRad(mSlopAngle));

	float move_along_nor_by_size = shape_half_sizes.Y() +  (2 * (shape_half_sizes.Y() - (shape_half_sizes.Y() * vx::VxCos(vx::DegToRad(mSlopAngle)))));
	//float height_above_ground = 18.0f/*6.0f*/ * (mSlopAngle / 22.5f);//5.0f;
	float height_above_ground = (offset_center_along_plane_sur_tan * vx::VxSin(vx::DegToRad(mSlopAngle))) + move_along_nor_by_size;

	float start_z = -15.0f;
	float z_stride = 6.0f;
	float shape_grp_zOffset = 2.5f;

	for (int i = 0; i < 10; ++i)
	{
		dyn_bodies_settings.friction = 1.0f - (0.1f * i);



		float z_coord_base = start_z + (z_stride * i);
		/// sphere 
		dyn_bodies_settings.debug_name = "sphere";
		dyn_bodies_settings.position = vx::Vec3(x_coord, height_above_ground, z_coord_base);
		dyn_bodies_settings.orientation = vx::Quat::Identity();
		dyn_bodies_settings.shape = sphere_shape;
		mPhysicsWorld->CreateBody(dyn_bodies_settings);

		/// box
		dyn_bodies_settings.debug_name = "box";
		dyn_bodies_settings.position = vx::Vec3(x_coord, height_above_ground, z_coord_base + shape_grp_zOffset);
		dyn_bodies_settings.shape = box_shape;
		dyn_bodies_settings.orientation = orientation;
		mPhysicsWorld->CreateBody(dyn_bodies_settings);
	}
}

void FrictionScenario22_5::Init(vx::PhysicsWorld* i_world)
{
	FrictionScenario::Init(i_world);
	///cam 
	/// -16.92, 11.09, 10.015
	/// 
	if (mAppCamera)
	{
		vx::Vec3 pos = vx::Vec3(-10.46, 32.76f, 72.77f);
		vx::Vec3 fwd = vx::Vec3(0.65f, -0.34f, -0.68f);
		vx::Vec3 up = vx::Vec3(0.23f, 0.94f, -0.25f);
		Camera::State cam_state = Camera::State(pos, fwd, up);
		mAppCamera->SetState(cam_state);
	}

}

void FrictionScenario45::Init(vx::PhysicsWorld* i_world)
{
	FrictionScenario::Init(i_world);
	///cam 
	/// -16.92, 11.09, 10.015
	/// 
	if (mAppCamera)
	{
		vx::Vec3 pos = vx::Vec3(-10.46, 32.76f, 72.77f);
		vx::Vec3 fwd = vx::Vec3(0.74f, -0.313f, -0.66f);
		vx::Vec3 up = vx::Vec3(0.09f, 0.99f, -0.09f);
		Camera::State cam_state = Camera::State(pos, fwd, up);
		mAppCamera->SetState(cam_state);
	}

}
