#include "JointScenario.h"

#include "PhysicsWorld.h"

#include "Collision/Shapes/Shape.h"
#include "Collision/Shapes/SphereShape.h"
#include "Collision/Shapes/BoxShape.h"
#include "Collision/Shapes/CapsuleShape.h"

#include "external/imgui/imgui.h"

void RopeSetting(vx::DistanceConstraint& constraint)
{
	constraint.mMinDistance = 2.0f;
	constraint.mMaxDistance = 6.0f;
	constraint.mSpring.tunningMode = vx::ESpringTuningMode::StiffnessSoftness;
	constraint.mSpring.stiffness = 0.0f;
}

void SuspensionShockSettingCriticalDamping(vx::DistanceConstraint& constraint)
{
	///constraint.mMinDistance = -constraint.mRestLength;
	//constraint.mMaxDistance = constraint.mRestLength;
	constraint.mSpring.tunningMode = vx::ESpringTuningMode::FrequencyDamping;
	constraint.mSpring.frequency = 2.0f;
	constraint.mSpring.dampingRatio = 0.1f;
}
void HardBarSetting(vx::DistanceConstraint& constraint)
{
	//SuspensionShockSettingCriticalDamping(constraint);
	//return;
	//constraint.mMinDistance = -constraint.mRestLength;
	constraint.mMaxDistance = 2.5f;// constraint.mRestLength * 2.0f;
	constraint.mMinDistance = 2.5f;// constraint.mRestLength * 2.0f;
	constraint.mSpring.tunningMode = vx::ESpringTuningMode::StiffnessSoftness;
	constraint.mSpring.stiffness = 0.0f;
	constraint.mSpring.damping = 0.0f;
}


void SuspensionShockSetting(vx::DistanceConstraint& constraint)
{
	//constraint.mMinDistance = -constraint.mRestLength;
	//constraint.mMaxDistance = constraint.mRestLength;
	constraint.mSpring.tunningMode = vx::ESpringTuningMode::FrequencyDamping;
	constraint.mSpring.frequency = 2.0f;
	constraint.mSpring.dampingRatio = 2.0f;
}


void JointScenario::Init(vx::PhysicsWorld* i_world)
{

	vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();
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

	//hard bar 
	HardBarSetting(mJoint);


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
	//lets test placing the achor to the side of the capsule, instead of the origin
	new_joint.mLocalAnchorB = Vec3(0.0f, 1.0f, 0.0f);
	HardBarSetting(new_joint);

	dyn_bodies_settings.position = vx::Vec3(-2.0f, 5.5f, 0.0f);
	dyn_bodies_settings.debug_name = "box";
	dyn_bodies_settings.shape = unit_box;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	mNotInPipelineJoints.push_back(mJoint);
	auto& new_joint2 = mNotInPipelineJoints.back();
	new_joint2.mLocalAnchorA = Vec3(0.0f, -1.0f, 0.0f); //quick offset
	new_joint2.mBodyA = &mPhysicsWorld->GetBodies()[2];
	new_joint2.mBodyB = &mPhysicsWorld->GetBodies()[3];
	HardBarSetting(new_joint2);

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


//Constraint window
void ConstaintPanel(DistanceConstraint& constraint)
{
	if(constraint.mBodyA && constraint.mBodyB)
		ImGui::Text("Body A ID: %d \nBody B ID: %d", constraint.mBodyA->GetID(), constraint.mBodyB->GetID());

	ImGui::DragFloat3("Local Anchor A", &constraint.mLocalAnchorA[0]);
	ImGui::DragFloat3("Local Anchor B", &constraint.mLocalAnchorB[0]);
	
	ImGui::DragFloat("Min Distance", &constraint.mMinDistance);
	ImGui::DragFloat("Max Distance", &constraint.mMaxDistance);

	ImGui::Text("Accumulated Lambda: %d", constraint.mAccumulatedLambda);
	
	ImGui::SeparatorText("Spring Setting");
	auto& spring = constraint.mSpring;

	EditorImGui::Combo("Tuning Mode", spring.tunningMode, "Stiffness Softness\0""Frequency Damping\0""\0");
	if (spring.tunningMode == ESpringTuningMode::StiffnessSoftness)
	{
		ImGui::DragFloat("Stiffness [N/m]", &spring.stiffness);
		ImGui::DragFloat("Damping [Ns/m]", &spring.damping);
	}
	else
	{
		ImGui::DragFloat("Frequency [Hz]", &spring.frequency);
		ImGui::SliderFloat("Damping Ratio", &spring.dampingRatio, 0.0f, 1.0f);
	}
	ImGui::SliderFloat("Softness", &spring.softness, 0.0f, 1.0f);
}


void JointScenario::OnUI()
{
	Scenario::OnUI();


	if (ImGui::Begin("Joint Window"))
	{
		static bool use_new = true;

		if (use_new)
			ConstaintPanel(mJoint);
		else
		{
			ImGui::SliderFloat("Joint Rest length", &mJoint.mRestLength, 0.0f, 10.0f);
			ImGui::SliderFloat("Joint Damping Ratio", &mJoint.mDampingRatio, 0.0f, 1.0f);
			ImGui::DragFloat("Joint Stiffness", &mJoint.mStiffness);
		}

		int _idx = 1;
		for (auto& joint : mNotInPipelineJoints)
		{
			ImGui::Separator();
			ImGui::Spacing();
			ImGui::PushID(&joint);
			ImGui::Text("joint %d", _idx++);
			if (use_new)
				ConstaintPanel(joint);
			else
			{
				ImGui::SliderFloat("Joint Rest length", &joint.mRestLength, 0.0f, 10.0f);
				ImGui::SliderFloat("Joint Damping Ratio", &joint.mDampingRatio, 0.0f, 1.0f);
				ImGui::DragFloat("Joint Stiffness", &joint.mStiffness);
			}
			ImGui::PopID();
		}
	}
	ImGui::End();
}
