#include "JengaScenario.h"

#include "SampleFramework/Camera.h"
#include "Vortrix/Dynamics/Body/Body.h"
#include "Vortrix/Collision/Shapes/BoxShape.h"

void JengaScenario::Init(vx::PhysicsWorld* i_world)
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
	//half_extents = vx::Vec3(0.25, 0.5f, 1.5f);

	vx::BodySettings body_setting = vx::BodySettings::DefaultDynamicConstruct();
	body_setting.friction = 0.8f;
	body_setting.restitution = 0.05f;
	//body_setting.shape = new vx::BoxShape(half_extents);


		//float block_length = 2.5f;
	//float block_height = 0.3f;
	//float block_width = 0.7f;
	vx::BodySettings jenga_body_settings = body_setting; //cpy
	CreateJenga(jenga_body_settings, vx::Vec3(0.5f, 0.3f, 1.5f), 3, vx::Vec3(0.0f, 0.5f, -5.0f));
	CreateJenga(jenga_body_settings, vx::Vec3(0.5f, 0.3f, 1.5f), 5, vx::Vec3(0.0f, 1.5f, 0.0f));
	CreateJenga(jenga_body_settings, vx::Vec3(0.5f, 0.3f, 1.5f), 8, vx::Vec3(0.0f, 0.5f, 5.0f));
	CreateJenga(jenga_body_settings, vx::Vec3(0.5f, 0.3f, 1.5f), 12, vx::Vec3(0.0f, 0.5f, 10.0f));
	CreateJenga(jenga_body_settings, vx::Vec3(0.5f, 0.3f, 1.5f), 16, vx::Vec3(-7.0f, 0.5f, 5.0f));

	CreateGroundPlane(100.0f);
}
