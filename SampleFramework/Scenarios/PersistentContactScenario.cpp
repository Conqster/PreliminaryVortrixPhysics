#include "PersistentContactScenario.h"
#include "Vortrix/PhysicsWorld.h"

#include "Vortrix/Maths/ScalarMath.h"

#include "Vortrix/Collision/Shapes/Shape.h"
#include "Vortrix/Collision/Shapes/BoxShape.h"
#include "Vortrix/Collision/Shapes/CapsuleShape.h"
#include "Vortrix/Collision/Shapes/SphereShape.h"

#include "SampleFramework/Camera.h"

#include "external/imgui/imgui.h"

#include "Vortrix/Dynamics/Constraints/PointConstraint.h"

void PersistentContactScenario::Init(vx::PhysicsWorld* i_world)
{
	Scenario::Init(i_world);

	//vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();

	//dyn_bodies_settings.position = vx::Vec3(0.0f, 5.5f, 0.0f);
	//dyn_bodies_settings.debug_name = "box";
	//dyn_bodies_settings.shape = new vx::BoxShape(0.5f);
	//mPhysicsWorld->CreateBody(dyn_bodies_settings);



	vx::BodySettings static_bodies_settings = vx::BodySettings::DefaultStaticConstruct();

	vx::CapsuleShapeSettings capsule_settings(0.5f, 0.5f);
	capsule_settings.SetDensity(0.0f);

	static_bodies_settings.position = vx::Vec3(-2.0f, 5.5f, 0.0f);
	static_bodies_settings.debug_name = "capsule";
	static_bodies_settings.shape = vx::MakeRef<vx::CapsuleShape>(capsule_settings); // new vx::BoxShape(0.5f);
	static_bodies_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
	mPhysicsWorld->CreateBody(static_bodies_settings);

	vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();
	dyn_bodies_settings.position = vx::Vec3(-1.5f, 5.0f, 0.0f);
	//dyn_bodies_settings.position = vx::Vec3(1.0f, 4.0f, 0.0f);
	
	//dyn_bodies_settings.position = vx::Vec3(2.0f, 5.5f, 0.0f);
	//dyn_bodies_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
	dyn_bodies_settings.debug_name = "capsule";
	dyn_bodies_settings.shape = vx::MakeRef<vx::CapsuleShape>(0.5f, 0.5f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);


	//vx::DistanceConstraint joint = vx::DistanceConstraint(&mPhysicsWorld->GetBodies()[0], &mPhysicsWorld->GetBodies()[1], rope_constraint_settings);
	//joint.SetLocalAnchorA(vx::Vec3(0.0f, 0.5f, 0.0f));
	//joint.SetLocalAnchorB(vx::Vec3(0.0f, 0.5f, 0.0f));

	vx::PointConstraintSettings point_constraint_setting_ws;
	point_constraint_setting_ws.anchorPointFrame = vx::EConstraintFrame::World;
	point_constraint_setting_ws.anchorA = dyn_bodies_settings.position + vx::Vec3(0.0f, 0.5f, 0.0f);
	point_constraint_setting_ws.anchorB = dyn_bodies_settings.position + vx::Vec3(0.0f, 0.5f, 0.0f);

	vx::PointConstraintSettings point_constraint_setting;
	point_constraint_setting.anchorPointFrame = vx::EConstraintFrame::Local;
	point_constraint_setting.anchorA = vx::Vec3(0.0f, 1.1875f, 0.0f);
	point_constraint_setting.anchorB = vx::Vec3(0.0f, -1.1875f, 0.0f);

	//point_constraint_setting.enableVelocityBias = true;
	//point_constraint_setting.errorTreshold = 1.0f;

	////mNotInPipelineJoints.reserve(25);
	//auto& phys_constraint_coord = mPhysicsWorld->mConstraintCoordinator;
	//phys_constraint_coord->AddConstraintT(joint);

	//maybe every frame update worldanhor
	//mPhysicsWorld->mBallJoint = new vx::PointConstraint(&mPhysicsWorld->GetBodies()[0], &mPhysicsWorld->GetBodies()[1], point_constraint_setting);
	mTestConstraint = new vx::PointConstraint(&mPhysicsWorld->GetBodies()[0], &mPhysicsWorld->GetBodies()[1], point_constraint_setting);
	mPhysicsWorld->AddConstraint(mTestConstraint);

	point_constraint_setting.anchorA = vx::Vec3(0.0f, 1.0f, 0.0f);
	point_constraint_setting.anchorB = vx::Vec3(0.0f, -1.0f, 0.0f);


	//offset next test to the right 
	vx::Vec3 offset = vx::Vec3(4.0f, 0.0f, 0.0f);
	static_bodies_settings.position += offset;
	mPhysicsWorld->CreateBody(static_bodies_settings);



	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	vx::PointConstraint pt_constraint2 = vx::PointConstraint(&mPhysicsWorld->GetBodies()[2], &mPhysicsWorld->GetBodies()[3], point_constraint_setting);

	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	vx::PointConstraint pt_constraint3 = vx::PointConstraint(&mPhysicsWorld->GetBodies()[1], &mPhysicsWorld->GetBodies()[4], point_constraint_setting);

	//constraint between a cube and sphere 
	dyn_bodies_settings.position += offset;
	dyn_bodies_settings.debug_name = "box";
	dyn_bodies_settings.shape = vx::MakeRef<vx::BoxShape>(0.5f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.debug_name = "sphere";
	dyn_bodies_settings.shape = vx::MakeRef<vx::SphereShape>(0.5f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	point_constraint_setting.anchorA = vx::Vec3(0.0f, 1.0f, 0.0f);
	point_constraint_setting.anchorB = vx::Vec3(0.0f, 0.0f, 0.0f);

	

	vx::PointConstraint pt_joints[3] = {
		pt_constraint2,
		pt_constraint3,
		vx::PointConstraint((&mPhysicsWorld->GetBodies().back())-1, &mPhysicsWorld->GetBodies().back(), point_constraint_setting)
	};
	mPhysicsWorld->CreateConstraintsT(pt_joints, 3);


	/// Ground plane
	CreateGroundPlane(100.0f);
}

void PersistentContactScenario::PostPhysicsStep(float dt)
{
	Scenario::PostPhysicsStep(dt);
	//mPhysicsWorld->mBallJoint->ResolveOffset();
}

void PersistentContactScenario::OnClose()
{
	Scenario::OnClose();
}

PersistentContactScenario::~PersistentContactScenario()
{
	Scenario::OnClose();
	
	//dontr use desconstructor here
	// as Os would reclaim mem anyway for now 
	//OnClose();
}

void PersistentContactScenario::OnUI()
{
	Scenario::OnUI();

	if (mPhysicsWorld == nullptr)
		return;


	//contact contraints
	auto& contact_constraint_stat = mPhysicsWorld->GetContactConstraintSolverStats();

	if (ImGui::Begin("PersistentContactScenario Window"))
	{
		vx::StackString<32> text;
		ImGui::Text("Last Step Bias: %s", text.Data());

		if (mTestConstraint)
		{
			ImGui::Text("Constraint Idx: %d", mTestConstraint->ConstraintIdx());
			ImGui::SameLine();
			if(mTestConstraint->ConstraintIdx() != vx::Constraint::kInvalidIdx)
			{
				if (ImGui::Button("Remove"))
					mPhysicsWorld->RemoveConstraint(mTestConstraint);
			}
			else
			{
				if (ImGui::Button("Add"))
					mPhysicsWorld->AddConstraint(mTestConstraint);
			}
		}


		ImGui::Text("Num of Contacts: %d", contact_constraint_stat.numContactConstraints);
		ImGui::Text("Num of Persistnet Contacts: %d", contact_constraint_stat.numPersistentContact);

		ImGui::Text("Num of Persistent Point: %d", contact_constraint_stat.actualPointCounts);
		ImGui::Text("Num of Valid Persistent Point: %d", contact_constraint_stat.actualPersistentPointCounts);

		ImGui::Text("Total Nor Lambda: %f", contact_constraint_stat.totalNorLambda);
		ImGui::Text("Total Tan Lambda: %f", contact_constraint_stat.totalTanLambda);
		ImGui::Text("Total BiTan Lambda: %f", contact_constraint_stat.totalBiTanLambda);
	}
	ImGui::End();
}

