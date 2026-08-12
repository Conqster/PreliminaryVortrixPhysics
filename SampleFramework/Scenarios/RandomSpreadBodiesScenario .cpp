#include "Vortrix/PhysicsWorld.h"
#include "RandomSpreadBodiesScenario.h"
#include "Vortrix/Collision/Shapes/BoxShape.h"
#include "Vortrix/Collision/Shapes/CapsuleShape.h"

void RandomSpreadBoxBodiesScenario::Init(vx::PhysicsWorld* i_world)
{
	RandomSpreadBodiesScenario::Init(i_world);

	const float max_range = 190.0f * 0.7f;

	int stack_count = 3;

	float min_height = 1.0f;
	float half_size = 0.5f;
	float stack_offset = half_size * 2;// *2;

	vx::BoxShapeSettings shape_settings(half_size);
	//vx::CapsuleShapeSettings shape_settings(half_size, half_size);

	vx::BodySettings bodies_settings = vx::BodySettings::DefaultDynamicConstruct();
	bodies_settings.debug_name = "box";
	bodies_settings.shape = vx::MakeRef<vx::BoxShape>(shape_settings);
	//bodies_settings.shape = vx::MakeRef<vx::CapsuleShape>(shape_settings);

	for (vx::uint32 i = 0; i < 300; ++i)
	{
		vx::Vec3 v = vx::Vec3(vx::Random::Float(-max_range, max_range),
			min_height, vx::Random::Float(-max_range, max_range));

		bodies_settings.position = v;
		mPhysicsWorld->CreateBody(bodies_settings);
		for (vx::uint32 j = 0; j < 2; ++j)
		{
			v.SetY(stack_offset + v.Y());
			bodies_settings.position = v;
			mPhysicsWorld->CreateBody(bodies_settings);
		}
	}
}