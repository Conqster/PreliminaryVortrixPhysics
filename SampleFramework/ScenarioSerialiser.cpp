#include "ScenarioSerialiser.h"

#include "SampleFramework/Camera.h"

#include "Vortrix/Dynamics/Body/BodyManager.h"

#include "Vortrix/Collision/Shapes/SphereShape.h"
#include "Vortrix/Collision/Shapes/BoxShape.h"
#include "Vortrix/Collision/Shapes/CapsuleShape.h"
#include "Vortrix/Collision/Shapes/PlaneShape.h"



#include "Vortrix/PhysicsWorld.h"

#include <filesystem>

#include <ostream>

bool serialiser::Serialise(const vx::BodyManager* body_manager, const Camera& app_cam, const char* name, const char* directory, const char* info)
{

	std::vector<SerialisedShape> shapes;
	std::vector<SerialisedBody> bodies;

	shapes.reserve(body_manager->BodyCount());
	bodies.reserve(body_manager->BodyCount());

	std::vector<uint32_t> body_id_to_serialied_idx;

	//body_id_to_serialied_idx.reserve(body_manager->BodyCount() + body_manager->mFreedIdxs.size());

	for (const auto& body : body_manager->GetBodies())
	{
		if (!body.IsIDValid()) continue;

		const vx::Shape* shape = body.GetShape();

		vx::Float3 planeNormal = vx::Float3(0.0f);
		if (shape->Type() == vx::EShapeType::Plane)
			planeNormal = static_cast<const vx::PlaneShape*>(shape)->Normal().ToFloat3();

		shapes.push_back({ 
			shape->Type(),
			shape->HalfExtents().ToFloat3(),
			planeNormal,
			shape->Density()
			});

		bodies.push_back(
			{
				//"",
				vx::uint32(shapes.size() - 1),
				body.Position().ToFloat3(),
				body.Orientation(),
				body.FrictionCoeff(),
				body.RestitutionCoeff(),
				body.MotionType(),
				body.MaxLinearVelocity(),
				body.MaxAngularVelocity(),
				body_manager->GetBodyDebugInfo(body.GetID()).bodyInBroadphase,
				body.LinearDamping(),
				body.AngularDamping()
			});
	}


	SerialisedSceneCameraState serialise_cam
	{
		app_cam.Position().ToFloat3(),
		app_cam.Forward().ToFloat3(),
		app_cam.Up().ToFloat3()
	};

	Json j;
	j["Name"] = name;
	j["Info"] = info;
	j["Shapes"] = ToJson(shapes);
	j["Bodies"] = ToJson(bodies);
	j["Camera"] = ToJson(serialise_cam);
		 
    std::filesystem::path _path = directory;
    auto full_path = std::filesystem::absolute(_path);
	if(!std::filesystem::exists(_path))
		std::filesystem::create_directories(_path);
	std::ofstream o(_path.string() + "\\" + name + ".json");
	o << std::setw(4) << j << std::endl;
    return true;
}


bool serialiser::Deserialise(vx::PhysicsWorld* io_physicsworld, std::string& scene_name, std::string& scene_info, Camera& app_cam, const char* file_path)
{
	std::filesystem::path _file_path = file_path;

	if (!std::filesystem::is_regular_file(file_path) || _file_path.extension() != ".json")
	{
		VX_LOG_WARN("Serialising Game; Failed; not a file or invalid path");
		return false;
	}

	std::ifstream i(_file_path.string());
	Json j;

	try
	{
		i >> j;
	}
	catch (Json::parse_error& e)
	{
		VX_LOG_DEBUG("Serialising Game; Failed", e.what());
		return false;
	}



	std::vector<SerialisedShape> shapes;
	std::vector<SerialisedBody> bodies;

	SerialisedSceneCameraState _cam;

	try
	{
		scene_name = serialiser::FromJson<std::string>(j["Name"]);
		scene_info = serialiser::FromJson<std::string>(j["Info"]);

		_cam = serialiser::FromJson<SerialisedSceneCameraState>(j["Camera"]);

		shapes = serialiser::FromJson<std::vector<SerialisedShape>>(j["Shapes"]);
		bodies = serialiser::FromJson<std::vector<SerialisedBody>>(j["Bodies"]);
	}
	catch (Json::exception& e)
	{
		VX_LOG_DEBUG("Serialising Game; Failed", e.what());
		return false;
	}


	Camera::State cam_state = Camera::State(
		vx::Vec3::LoadFloat3Raw(_cam.position),
		vx::Vec3::LoadFloat3Raw(_cam.forward),
		vx::Vec3::LoadFloat3Raw(_cam.up));
	app_cam.SetState(cam_state);

	std::vector<vx::RefConst<vx::Shape>> body_shapes;
	body_shapes.reserve(shapes.size());
	///later create all bodies at once


	for (vx::uint32 i = 0; i < bodies.size(); ++i)
	{
		const SerialisedShape& serialised_shape = shapes[i];

		vx::Ref<vx::Shape> shape_ptr = vx::MakeRef<vx::SphereShape>(0.5f);
		switch (serialised_shape.type)
		{
		case vx::EShapeType::Sphere:
		{
			vx::SphereShapeSettings setting(serialised_shape.halfExtext.x);
			setting.SetDensity(serialised_shape.density);
			shape_ptr = vx::MakeRef<vx::SphereShape>(setting);
		}
			break;
		case vx::EShapeType::Box:
		{
			vx::BoxShapeSettings setting(vx::Vec3::LoadFloat3Raw(serialised_shape.halfExtext));
			setting.SetDensity(serialised_shape.density);
			shape_ptr = vx::MakeRef<vx::BoxShape>(setting);
		}
			break;
		case vx::EShapeType::Capsule:
		{
			float cylinder_half_height = serialised_shape.halfExtext.y - serialised_shape.halfExtext.x;
			vx::CapsuleShapeSettings setting(serialised_shape.halfExtext.x, cylinder_half_height);
			setting.SetDensity(serialised_shape.density);
			shape_ptr = vx::MakeRef<vx::CapsuleShape>(setting);
		}
			break;
		case vx::EShapeType::Plane:
		{
			vx::PlaneShapeSettings setting(vx::Vec3::LoadFloat3Raw(serialised_shape.planeNormal), serialised_shape.halfExtext.x);
			setting.SetDensity(serialised_shape.density);
			shape_ptr = vx::MakeRef<vx::PlaneShape>(setting);
		}
			break;
		default:
			VX_ASSERT(false);
			break;
		}
		body_shapes.push_back(shape_ptr);

		const SerialisedBody& serialised_body = bodies[i];

		vx::BodySettings body_settings;
		vx::uint32 serialisedShapeIndex = vx::uint32(-1);
		body_settings.position = vx::Vec3::LoadFloat3Raw(serialised_body.position);
		body_settings.orientation = serialised_body.orientation;
		body_settings.friction = serialised_body.friction;
		body_settings.restitution = serialised_body.restitution;
		body_settings.motionType = serialised_body.motionType;
		body_settings.maxLinearVelocity = serialised_body.maxLinearVelocity;
		body_settings.maxAngularVelocity = serialised_body.maxAngularVelocity;
		/// if in broadphase body participates in collision
		body_settings.inBroadphase = serialised_body.inBroadphase;
		body_settings.linearDamping = serialised_body.linearDamping;
		body_settings.angularDamping = serialised_body.angularDamping;

		body_settings.shape = body_shapes[serialised_body.serialisedShapeIndex];

		///later create all bodies at once
		io_physicsworld->CreateBody(body_settings);
	}
}

