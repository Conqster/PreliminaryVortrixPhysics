#include "JointScenario.h"

#include "PhysicsWorld.h"

#include "Collision/Shapes/Shape.h"
#include "Collision/Shapes/SphereShape.h"
#include "Collision/Shapes/BoxShape.h"
#include "Collision/Shapes/CapsuleShape.h"

#include "external/imgui/imgui.h"

vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();

void JointScenario::Init(vx::PhysicsWorld* i_world)
{

	mPhysicsWorld = i_world;
	VX_ASSERT(mPhysicsWorld, "Physics World is null");



	vx::SphereShape* unit_sphere = new vx::SphereShape(0.5f);
	dyn_bodies_settings.position = vx::Vec3(3.0f, 5.5f, 0.0f);
	dyn_bodies_settings.debug_name = "sphere";
	dyn_bodies_settings.shape = unit_sphere;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	vx::BoxShape* unit_box = new vx::BoxShape(0.5f);
	dyn_bodies_settings.position = vx::Vec3(-2.0f, 5.5f, 0.0f);
	dyn_bodies_settings.debug_name = "box";
	dyn_bodies_settings.shape = unit_box;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	mJoint.mBodyA = &mPhysicsWorld->GetBodies()[0];
	mJoint.mBodyB = &mPhysicsWorld->GetBodies()[1];

	mJoint.mRestLength = 2.5f;
	mJoint.mDampingRatio = 0.3f;
	mJoint.mStiffness = 2500.0f;

	mPhysicsWorld->mTestJoint = &mJoint;

	vx::CapsuleShape* unit_capsule = new vx::CapsuleShape(0.5f, 0.5f);
	dyn_bodies_settings.position = vx::Vec3(-2.0f, 5.5f, 0.0f);
	dyn_bodies_settings.debug_name = "capsule";
	dyn_bodies_settings.shape = unit_capsule;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	mNotInPipelineJoints.push_back(mJoint);
	auto& new_joint = mNotInPipelineJoints.back();
	new_joint.mBodyA = &mPhysicsWorld->GetBodies()[1];
	new_joint.mBodyB = &mPhysicsWorld->GetBodies()[2];


	dyn_bodies_settings.position = vx::Vec3(-2.0f, 5.5f, 0.0f);
	dyn_bodies_settings.debug_name = "box";
	dyn_bodies_settings.shape = unit_box;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	mNotInPipelineJoints.push_back(mJoint);
	auto& new_joint2 = mNotInPipelineJoints.back();
	new_joint2.mBodyA = &mPhysicsWorld->GetBodies()[2];
	new_joint2.mBodyB = &mPhysicsWorld->GetBodies()[3];

	/// Ground plane
	CreateGroundPlane(100.0f);
}

void JointScenario::PostPhysicsStep(float dt)
{
	for(auto& joint : mNotInPipelineJoints)
		joint.Solve(dt);

	Scenario::PostPhysicsStep(dt);

	if (mDebugGizmos)
	{
		for (const auto& joint : mNotInPipelineJoints)
			joint.DebugGizmos(mDebugGizmos);
	}
}

void JointScenario::OnUI()
{
	Scenario::OnUI();

	if (ImGui::Begin("Joint Window"))
	{
		ImGui::SliderFloat("Joint Rest length", &mJoint.mRestLength, 0.0f, 10.0f);
		ImGui::SliderFloat("Joint Damping Ratio", &mJoint.mDampingRatio, 0.0f, 1.0f);
		ImGui::DragFloat("Joint Stiffness", &mJoint.mStiffness);

		int _idx = 1;
		for (auto& joint : mNotInPipelineJoints)
		{
			ImGui::PushID(&joint);
			ImGui::Text("joint %d", _idx++);
			ImGui::SliderFloat("Joint Rest length", &joint.mRestLength, 0.0f, 10.0f);
			ImGui::SliderFloat("Joint Damping Ratio", &joint.mDampingRatio, 0.0f, 1.0f);
			ImGui::DragFloat("Joint Stiffness", &joint.mStiffness);
			ImGui::PopID();
		}
	}
	ImGui::End();
}
