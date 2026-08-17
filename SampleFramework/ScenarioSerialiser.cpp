#include "ScenarioSerialiser.h"

#include "SampleFramework/Camera.h"

#include "Vortrix/Dynamics/Body/BodyManager.h"

#include "Vortrix/Collision/Shapes/SphereShape.h"
#include "Vortrix/Collision/Shapes/BoxShape.h"
#include "Vortrix/Collision/Shapes/CapsuleShape.h"
#include "Vortrix/Collision/Shapes/PlaneShape.h"

#include "SampleFramework/Scenarios/Scenario.h"


#include "Vortrix/Dynamics/Constraints/PointConstraint.h"

#include "Vortrix/PhysicsWorld.h"

#include <filesystem>

#include <ostream>

#include <unordered_set>


////when serialising scenario remove ragdoll bodies before saving
/// so ragdolls are saved as ragdoll setting; easier for batch body shape loading
/// also helps with its constraints saving as well 


bool serialiser::Serialise(const vx::BodyManager* body_manager, const Camera& app_cam, const char* name, const char* directory, const char* info, Scenario* scenario)
{

	std::vector<SerialisedShape> shapes;
	std::vector<SerialisedBody> bodies;

	shapes.reserve(body_manager->BodyCount());
	bodies.reserve(body_manager->BodyCount());

	std::vector<uint32_t> body_id_to_serialied_idx;
	body_id_to_serialied_idx.resize(body_manager->BodyCount() + body_manager->CurrentFreeIndicesCount(), vx::uint32(-1));

	///remove ragdoll datas
	const auto& ragdolls_bodies = scenario->RagdollCreatedBodies();
	auto& ragdolls_constraints = scenario->RagdollCreatedConstraints();

	
	std::vector<vx::BodyID> bodies_id;
	for (const auto& body : body_manager->GetBodies())
		//if (body.IsIDValid())
			bodies_id.push_back(body.ID());

	if (!ragdolls_bodies.empty())
	{
		//scenario->PhysicsWorld()->RemoveBodies(&ragdolls_bodies[0], ragdolls_bodies.size());
		for(const auto& body_id : ragdolls_bodies)
			bodies_id[body_id.Idx()] = vx::BodyID();
	}
	//if(!ragdolls_constraints.empty())
		//scenario->PhysicsWorld()->RemoveConstraints(&ragdolls_constraints[0], ragdolls_constraints.size());

	vx::Constraints physics_constraints = scenario->PhysicsWorld()->NonContactConstraints();

	if (!ragdolls_constraints.empty())
	{
		std::unordered_set<vx::Constraint*> _remove(ragdolls_constraints.begin(), ragdolls_constraints.end());
		physics_constraints.erase(

			std::remove_if(physics_constraints.begin(), physics_constraints.end(),
				[&](vx::Constraint* x)
				{
					return _remove.find(x) != _remove.end();
				}),
			physics_constraints.end()
		);


		/// this is bad; good hack for now
		for (const auto& constraint : physics_constraints)
		{
			const vx::BodyID body_a = constraint->BodyA()->ID();
			const vx::BodyID body_b = constraint->BodyB()->ID();

			if (!bodies_id[body_a.Idx()].IsValid())
			{
				VX_LOG_WARN("Ragdoll group must have invalidate body id but still connected to another joint constraint, updating: ", body_a.ID());
				bodies_id[body_a.Idx()] = body_a;
			}

			if (!bodies_id[body_b.Idx()].IsValid())
			{
				VX_LOG_WARN("Ragdoll group must have invalidate body id, but still connected to another joint constraint, updating:", body_b.ID());
				bodies_id[body_b.Idx()] = body_b;
			}
		}
	}
	//if (!ragdolls_constraints.empty())
	//{
	//	for (vx::Constraint** c = &ragdolls_constraints[0], 
	//		**c_end = (&ragdolls_constraints[0]) + ragdolls_constraints.size();
	//		c < c_end; ++c)
	//	{
	//		VX_ASSERT((*c), "Attempting to remove null constraint");
	//		vx::Constraint::Idx c_idx = (*c)->ConstraintIdx();
	//		VX_ASSERT(c_idx != vx::Constraint::kInvalidIdx, "Attempting to remove invalid constraint");

	//		vx::Constraint::Idx c_idx_end = physics_constraints.back()->ConstraintIdx();

	//		//swap 
	//		if (c_idx < c_idx_end)
	//		{
	//			std::swap(physics_constraints[c_idx], physics_constraints[c_idx_end]);
	//			physics_constraints[c_idx]->ConstraintIdx(c_idx);
	//		}

	//		(*c)->ConstraintIdx(vx::Constraint::kInvalidIdx);
	//		physics_constraints.pop_back();
	//	}
	//}



	//for (const auto& body : body_manager->GetBodies())
	vx::uint32 serialised_body_count = 0;
	for(auto& body_id : bodies_id)
	{
		//if (!body.IsIDValid()) continue;
		if (!body_id.IsValid()) continue;
		const auto& body = body_manager->GetBody(body_id);

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
				body_manager->GetBodyDebugInfo(body.ID()).bodyInBroadphase,
				body.LinearDamping(),
				body.AngularDamping()
			});

		body_id_to_serialied_idx[body_id.Idx()] = serialised_body_count++;
	}


	std::vector<SerialisedDistanceConstraint> serialised_distance_constraint;
	std::vector<SerialisedPointConstraint> serialised_point_constraint;
	///serialising constraint
	if (!physics_constraints.empty())
	{
		for (const auto& constraint : physics_constraints)
		{
			VX_ASSERT(constraint->BodyA());
			VX_ASSERT(constraint->BodyB());
			vx::uint32 body_a = body_id_to_serialied_idx[constraint->BodyA()->ID().Idx()];
			vx::uint32 body_b = body_id_to_serialied_idx[constraint->BodyB()->ID().Idx()];
			VX_ASSERT(body_a != vx::uint32(-1));
			VX_ASSERT(body_b != vx::uint32(-1));

			switch (constraint->Type())
			{
			case vx::EConstraintType::Distance:
			{
				const vx::DistanceConstraint* dist_constraint = static_cast<const vx::DistanceConstraint*>(constraint);
				SerialisedDistanceConstraint serialise_constraint;
				serialise_constraint.serialisedBodyA = body_a;
				serialise_constraint.serialisedBodyB = body_b;
				serialise_constraint.localAnchorA = dist_constraint->LocalAnchorA();
				serialise_constraint.localAnchorB = dist_constraint->LocalAnchorB();
				serialise_constraint.minDist = dist_constraint->MinDistance();
				serialise_constraint.maxDist = dist_constraint->MaxDistance();
				serialise_constraint.frequency = dist_constraint->SpringFrequency();
				serialise_constraint.dampingRatio = dist_constraint->SpringDampingRatio();

				serialised_distance_constraint.push_back(serialise_constraint);
				break;
			}
			case vx::EConstraintType::Point:
			{
				const vx::PointConstraint* point_constraint = static_cast<const vx::PointConstraint*>(constraint);
				SerialisedPointConstraint serialise_constraint;
				serialise_constraint.serialisedBodyA = body_a;
				serialise_constraint.serialisedBodyB = body_b;

				serialise_constraint.localAnchorA = point_constraint->LocalAnchorA();
				serialise_constraint.localAnchorB = point_constraint->LocalAnchorB();


				serialise_constraint.enableVelocityBias = point_constraint->mHasVelocityBias;
				serialise_constraint.errorTreshold = point_constraint->mErrorTreshold;

				serialised_point_constraint.push_back(serialise_constraint);
				break;
			}
			default:
				VX_ASSERT(false);
				break;
			}
		}
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
	j["Distance Constraint"] = ToJson(serialised_distance_constraint);
	j["Point Constraint"] = ToJson(serialised_point_constraint);

	///update the positions of ragdoll using bodies approx
	for (auto& _ragdoll : scenario->RagdollSettings())
	{
		if (_ragdoll.firstBodyOffsetInScene == -1)
			continue;


		const int body_idx = _ragdoll.firstBodyOffsetInScene;
		const int body_count = _ragdoll.limbsCount;

		vx::Vec3 pos = vx::Vec3::Zero();

		//get first five bodies
		for (vx::uint32 i = body_idx; i < (body_idx + body_count); ++i)
			pos += body_manager->GetBody(scenario->RagdollCreatedBodies()[i]).Position();

		pos /= float(body_count);

		_ragdoll.position = pos;
	}

	j["Ragdolls"] = ToJson(scenario->RagdollSettings());
		 
	std::filesystem::path _path = directory;
	auto full_path = std::filesystem::absolute(_path);
	if(!std::filesystem::exists(_path))
		std::filesystem::create_directories(_path);
	std::ofstream o(_path.string() + "\\" + name + ".json");
	o << std::setw(4) << j << std::endl;
	return true;
}


bool serialiser::Deserialise(vx::PhysicsWorld* io_physicsworld, Scenario* scenario, std::string& scene_name, std::string& scene_info, Camera& app_cam, const char* file_path)
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
	std::vector<SerialisedRagdollSettings> ragdoll_settings;

	std::vector<SerialisedDistanceConstraint> serialised_distance_constraint;
	std::vector<SerialisedPointConstraint> serialised_point_constraint;

	SerialisedSceneCameraState _cam;

	try
	{
		scene_name = serialiser::FromJson<std::string>(j["Name"]);
		scene_info = serialiser::FromJson<std::string>(j["Info"]);

		_cam = serialiser::FromJson<SerialisedSceneCameraState>(j["Camera"]);

		shapes = serialiser::FromJson<std::vector<SerialisedShape>>(j["Shapes"]);
		bodies = serialiser::FromJson<std::vector<SerialisedBody>>(j["Bodies"]);

		if (scenario && j["Ragdolls"] != nullptr)
			ragdoll_settings = serialiser::FromJson<std::vector<SerialisedRagdollSettings>>(j["Ragdolls"]);


		if (j["Distance Constraint"] != nullptr)
			serialised_distance_constraint = serialiser::FromJson<std::vector<SerialisedDistanceConstraint>>(j["Distance Constraint"]);
		if (j["Point Constraint"] != nullptr)
			serialised_point_constraint = serialiser::FromJson<std::vector<SerialisedPointConstraint>>(j["Point Constraint"]);

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


	bool cache_body_id = !serialised_distance_constraint.empty() || !serialised_point_constraint.empty();

	std::vector<vx::BodyID> serialise_body_to_bodyID;
	serialise_body_to_bodyID.reserve(bodies.size());

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
		vx::Body* body = io_physicsworld->CreateBody(body_settings);


		(cache_body_id) ? serialise_body_to_bodyID.push_back(body->ID()) : void(0);
	}


	for (auto& constraint : serialised_distance_constraint)
	{
		VX_ASSERT(constraint.serialisedBodyA != vx::uint32(-1));
		VX_ASSERT(constraint.serialisedBodyB != vx::uint32(-1));
		vx::Body* body_a = &io_physicsworld->GetBodyManager().GetBody(serialise_body_to_bodyID[constraint.serialisedBodyA]);
		vx::Body* body_b = &io_physicsworld->GetBodyManager().GetBody(serialise_body_to_bodyID[constraint.serialisedBodyB]);


		vx::DistanceConstraintSettings _constraint;
		_constraint.localAnchorA = constraint.localAnchorA;
		_constraint.localAnchorB = constraint.localAnchorB;
		_constraint.minDist = constraint.minDist;
		_constraint.maxDist = constraint.maxDist;
		_constraint.frequency = constraint.frequency;
		_constraint.dampingRatio = constraint.dampingRatio;


		///later might want to hold pointers to constraints
		io_physicsworld->CreateConstraintsT(new vx::DistanceConstraint(body_a, body_b, _constraint), 1);
	}

	for (auto& constraint : serialised_point_constraint)
	{
		VX_ASSERT(constraint.serialisedBodyA != vx::uint32(-1));
		VX_ASSERT(constraint.serialisedBodyB != vx::uint32(-1));
		vx::Body* body_a = &io_physicsworld->GetBodyManager().GetBody(serialise_body_to_bodyID[constraint.serialisedBodyA]);
		vx::Body* body_b = &io_physicsworld->GetBodyManager().GetBody(serialise_body_to_bodyID[constraint.serialisedBodyB]);

		vx::PointConstraintSettings _constraint;

		_constraint.anchorA = constraint.localAnchorA;
		_constraint.anchorB = constraint.localAnchorB;
		_constraint.anchorPointFrame = vx::EConstraintFrame::Local;
		_constraint.enableVelocityBias = constraint.enableVelocityBias;
		_constraint.errorTreshold = constraint.errorTreshold;


		///later might want to hold pointers to constraints
		io_physicsworld->CreateConstraintsT(new vx::PointConstraint(body_a, body_b, _constraint), 1);
	}


	for (auto& ragdoll : ragdoll_settings)
		scenario->CreateRagdoll(ragdoll, nullptr);

}

