#include "BoxStackScenario.h"

#include "PhysicsWorld.h"
#include "Vortrix/Collision/Shapes/BoxShape.h"

#include "Camera.h"

void BoxStackScenario::Init(vx::PhysicsWorld* i_world)
{
	Scenario::Init(i_world);

	if (mAppCamera)
	{
		vx::Vec3 pos = vx::Vec3(-14.65, 12.30, -17.44);
		vx::Vec3 fwd = vx::Vec3(0.72, -0.32, 0.62);
		vx::Vec3 up = vx::Vec3(0.25, 0.95, 0.21);
		Camera::State cam_state = Camera::State(pos, fwd, up);
		mAppCamera->SetState(cam_state);
	}



	float half_size = 0.5f;
	vx::Vec3 half_extents = vx::Vec3(half_size);

	vx::BodySettings body_setting = vx::BodySettings::DefaultDynamicConstruct();
	body_setting.friction = 0.8f;
	body_setting.restitution = 0.05f;
	body_setting.linearDamping = 0.0f;
	body_setting.angularDamping = 0.0f;
	body_setting.shape = vx::MakeRef<vx::BoxShape>(half_extents);

	CreateBoxStack(body_setting, vx::Vec3(2), half_extents, vx::Vec3(0.0f, half_size, -15.0f));
	CreateBoxStack(body_setting, vx::Vec3(1, 2, 2), half_extents, vx::Vec3(-2.0f, half_size, -15.0f));

	CreateBoxStack(body_setting, vx::Vec3(4), half_extents, vx::Vec3(0.0f, half_size, -11.0f));
	CreateBoxStack(body_setting, vx::Vec3(1, 4, 4), half_extents, vx::Vec3(-3.0f, half_size, -11.0f));

	CreateBoxStack(body_setting, vx::Vec3(6), half_extents, vx::Vec3(0.0f, half_size, -3.0f));
	CreateBoxStack(body_setting, vx::Vec3(1, 6, 6), half_extents, vx::Vec3(-5.0f, half_size, -3.0f));

	CreateBoxStack(body_setting, vx::Vec3(8), half_extents, vx::Vec3(0.0f, half_size, 10.0f));

	CreateBoxStack(body_setting, vx::Vec3(8, 6, 1), half_extents, vx::Vec3(0.0f, half_size, 25.0f));


	//body_setting.shape = new vx::BoxShape(half_extents * vx::Vec3(2.0f, 1.0f, 0.25f));
	CreateBoxStack(body_setting, vx::Vec3(1, 10, 1), half_extents, vx::Vec3(10.0f, half_size, 5.0f));


	half_extents = vx::Vec3(0.25f);
	body_setting.shape = vx::MakeRef<vx::BoxShape>(half_extents);
	CreateBoxStack(body_setting, vx::Vec3(1, 10, 10), half_extents, vx::Vec3(10.0f, 0.25f, -15.0f));
	
	CreateGroundPlane(100.0f);
}


