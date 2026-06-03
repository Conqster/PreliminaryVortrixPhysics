#include "PersistentContactScenario.h"
#include "PhysicsWorld.h"

#include "Vortrix/Maths/ScalarMath.h"

#include "Collision/Shapes/Shape.h"
#include "Collision/Shapes/BoxShape.h"
#include "Collision/Shapes/CapsuleShape.h"

#include "Camera.h"

#include "external/imgui/imgui.h"

#include "Dynamics/PointConstraint.h"

void PersistentContactScenario::Init(vx::PhysicsWorld* i_world)
{
	mPhysicsWorld = i_world;
	VX_ASSERT(i_world, "Physics World is null");

	//vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();

	//dyn_bodies_settings.position = vx::Vec3(0.0f, 5.5f, 0.0f);
	//dyn_bodies_settings.debug_name = "box";
	//dyn_bodies_settings.shape = new vx::BoxShape(0.5f);
	//mPhysicsWorld->CreateBody(dyn_bodies_settings);



	vx::BodySettings static_bodies_settings = vx::BodySettings::DefaultStaticConstruct();

	static_bodies_settings.position = vx::Vec3(-2.0f, 5.5f, 0.0f);
	static_bodies_settings.debug_name = "capsule";
	static_bodies_settings.shape = new vx::CapsuleShape(0.5f, 0.5f); // new vx::BoxShape(0.5f);
	static_bodies_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
	mPhysicsWorld->CreateBody(static_bodies_settings);

	vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();
	dyn_bodies_settings.position = vx::Vec3(-1.5f, 5.0f, 0.0f);
	//dyn_bodies_settings.position = vx::Vec3(1.0f, 4.0f, 0.0f);
	
	//dyn_bodies_settings.position = vx::Vec3(2.0f, 5.5f, 0.0f);
	//dyn_bodies_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
	dyn_bodies_settings.debug_name = "capsule";
	dyn_bodies_settings.shape = new vx::CapsuleShape(0.5f, 0.5f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);


	//vx::DistanceConstraint joint = vx::DistanceConstraint(&mPhysicsWorld->GetBodies()[0], &mPhysicsWorld->GetBodies()[1], rope_constraint_settings);
	//joint.SetLocalAnchorA(vx::Vec3(0.0f, 0.5f, 0.0f));
	//joint.SetLocalAnchorB(vx::Vec3(0.0f, 0.5f, 0.0f));

	vx::PointConstraintSettings point_constraint_setting_ws;
	point_constraint_setting_ws.anchorPointFrame = vx::EConstraintFrame::World;
	point_constraint_setting_ws.anchorA = dyn_bodies_settings.position + Vec3(0.0f, 0.5f, 0.0f);
	point_constraint_setting_ws.anchorB = dyn_bodies_settings.position + Vec3(0.0f, 0.5f, 0.0f);

	vx::PointConstraintSettings point_constraint_setting;
	point_constraint_setting.anchorPointFrame = vx::EConstraintFrame::Local;
	point_constraint_setting.anchorA = Vec3(0.0f, 1.0f, 0.0f);
	point_constraint_setting.anchorB = Vec3(0.0f, -1.0f, 0.0f);
	//point_constraint_setting.enableVelocityBias = true;
	//point_constraint_setting.errorTreshold = 1.0f;

	////mNotInPipelineJoints.reserve(25);
	//auto& phys_constraint_coord = mPhysicsWorld->mConstraintCoordinator;
	//phys_constraint_coord->AddConstraintT(joint);

	//maybe every frame update worldanhor
	mPhysicsWorld->mBallJoint = new vx::PointConstraint(&mPhysicsWorld->GetBodies()[0], &mPhysicsWorld->GetBodies()[1], point_constraint_setting);
	//mPhysicsWorld->mBallJoint = new vx::PointConstraint(&mPhysicsWorld->GetBodies()[0], &mPhysicsWorld->GetBodies()[1], dyn_bodies_settings.position + Vec3(-1.0f, 0.0f, 0.0f));



	//offset next test to the right 
	Vec3 offset = Vec3(4.0f, 0.0f, 0.0f);
	static_bodies_settings.position += offset;
	mPhysicsWorld->CreateBody(static_bodies_settings);

	dyn_bodies_settings.position += offset;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	mPhysicsWorld->mBallJoint2 = new vx::PointConstraint(&mPhysicsWorld->GetBodies()[2], &mPhysicsWorld->GetBodies()[3], point_constraint_setting);


	/// Ground plane
	CreateGroundPlane(100.0f);
}

void PersistentContactScenario::PostPhysicsStep(float dt)
{
	Scenario::PostPhysicsStep(dt);
	//mPhysicsWorld->mBallJoint->ResolveOffset();
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
		text << mPhysicsWorld->mBallJoint2->mLastStepBias;
		ImGui::Text("Last Step Bias: %s", text.Data());


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

