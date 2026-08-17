#include "BoxPyramidStackScenario.h"

#include "Vortrix/PhysicsWorld.h"
#include "Vortrix/Collision/Shapes/BoxShape.h"

#include "SampleFramework/Camera.h"




void BoxPyramidStackScenario::Init(vx::PhysicsWorld* i_world)
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
	body_setting.shape = vx::MakeRef<vx::BoxShape>(half_extents);

	//CreateBoxPyramidStack(body_setting, 12, 12, 20, 1.0f, vx::Vec3(0.0f, 0.5f, 0.0f));
	//CreateBoxPyramidStack(body_setting, 12, 12, 20, half_extents, vx::Vec3(0.0f, 0.5f, 0.0f));
	//Create1DBoxPyramidStack(body_setting, 10, 1, 10, half_extents, vx::Vec3(0.0f, 0.5f, 0.0f), vx::kAxisX);
	//Create1DBoxPyramidStack(body_setting, 1, 6, 6, half_extents, vx::Vec3(0.0f, 0.5f, 0.0f), vx::kAxisZ);

	StructureConfig pyramid1D;
	pyramid1D.count = vx::Vec3(1, 10, 10);
	pyramid1D.halfExtent = half_extents;
	pyramid1D.basePos = vx::Vec3(0.0f, 0.5f, 0.0f);
	pyramid1D.GetSize = [&](int layer)
		{
			vx::Vec3& c = pyramid1D.count;
			return vx::Vec2(c.X(), c.Z()) - vx::Vec2(0.0f, layer);
		};
	CreateStructure(body_setting, pyramid1D, vx::Quat::Identity());
	pyramid1D.count = vx::Vec3(1, 10, 20);
	//pyramid1D.halfExtent = half_extents;
	pyramid1D.basePos = vx::Vec3(-10.0f, 0.5f, 0.0f);
	pyramid1D.GetSize = [&](int layer)
		{
			vx::Vec3& c = pyramid1D.count;
			return vx::Vec2(c.X(), c.Z()) - vx::Vec2(0.0f, layer);
		};
	CreateStructure(body_setting, pyramid1D, vx::Quat::Identity());
	pyramid1D.count = vx::Vec3(1, 15, 30);
	pyramid1D.halfExtent = vx::Vec3(0.7f, 0.5f, 1.0f) * 2.0f;
	pyramid1D.basePos = vx::Vec3(-20.0f, 1.0f, 0.0f);
		pyramid1D.GetSize = [&](int layer)
		{
			vx::Vec3& c = pyramid1D.count;
			return vx::Vec2(c.X(), c.Z()) - vx::Vec2(0.0f, layer);
		};
	vx::BodySettings _body_settings = body_setting;
	_body_settings.shape = vx::MakeRef<vx::BoxShape>(pyramid1D.halfExtent);
	CreateStructure(_body_settings, pyramid1D, vx::Quat::Identity());


	StructureConfig wall;
	wall.count = vx::Vec3(10, 4, 1);
	wall.basePos = vx::Vec3(10.0f, 2.9f, 10.0f);
	wall.halfExtent = vx::Vec3(0.5f);
	CreateStructure(body_setting, wall, vx::Quat::Identity());

	wall.count = vx::Vec3(1, 4, 10);
	wall.basePos = vx::Vec3(15.5f, 2.9f, 14.5f);
	CreateStructure(body_setting, wall, vx::Quat::Identity());


	StructureConfig ziggurat;
	//pyramid.count = vx::Vec3(2, 12, 12);
	ziggurat.count = vx::Vec3(8, 20, 8);
	ziggurat.basePos = vx::Vec3(10.0f, 0.6f, 0.0f);
	ziggurat.halfExtent = half_extents;
	ziggurat.GetSize = [&](int layer)
		{
			const vx::Vec3& c = ziggurat.count;
			int step = layer * 0.5f;
			return vx::Vec2(c.X(), c.Z()) - vx::Vec2(step, step);
		};
	CreateStructure(body_setting, ziggurat, vx::Quat::Identity());



	StructureConfig pyramid;
	//pyramid.count = vx::Vec3(2, 12, 12);
	pyramid.count = vx::Vec3(8);
	pyramid.basePos = vx::Vec3(10.0f, 1.5f, -10.0f);
	pyramid.halfExtent = half_extents;
	pyramid.GetSize = [&](int layer)
	{
		vx::Vec3& c = pyramid.count;
		return vx::Vec2(c.X(), c.Z()) - vx::Vec2(layer, layer);
	};
	CreateStructure(body_setting, pyramid, vx::Quat::Identity());


	StructureConfig ramp;
	ramp.count = vx::Vec3(10, 5, 3);
	ramp.basePos = vx::Vec3(10.0f, 1.5f, -18.0f);
	ramp.halfExtent = half_extents;
	ramp.GetSize = [&](int layer)
	{
		vx::Vec3& c = ramp.count;
		return vx::Vec2(c.X(), c.Z()) - vx::Vec2(layer, 0);
	};
	ramp.GetOffset = [&](int layer)
	{
		return vx::Vec3(layer * 0.5f, 0, 0);
	};
	body_setting.motionType = vx::EMotionType::Static;
	CreateStructure(body_setting, ramp, vx::Quat::Identity());

	StructureConfig ramp1;
	ramp1.count = vx::Vec3(10, 5, 3);
	ramp1.basePos = vx::Vec3(10.0f, 1.5f, -22.0f);
	ramp1.halfExtent = half_extents;
	ramp1.GetSize = [&](int layer)
		{
			vx::Vec3& c = ramp1.count;
			return vx::Vec2(c.X(), c.Z()) - vx::Vec2(layer, 0);
		};
	ramp1.GetOffset = [&](int layer)
		{
			return vx::Vec3(layer * 0.5f, 0, 0);
		};
	body_setting.motionType = vx::EMotionType::Dynamic;
	CreateStructure(body_setting, ramp1, vx::Quat::Identity());


	CreateGroundPlane(100.0f);


	//StructureConfig hollow;
	//hollow.count = vx::Vec3(6);
	//hollow.basePos = vx::Vec3(0.0f, 1.0f, 0.0f);
	//hollow.halfExtent = vx::Vec3(0.5f);
	//hollow.PlaceRule = [](int x, int y, int z, int w, int d)
	//{
	//	return (x = 0 || z == w - 1 || z == 0 || z == d - 1);
	//};
	//CreateStructure(body_setting, hollow);

}


