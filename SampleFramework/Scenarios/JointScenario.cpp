#include "JointScenario.h"

#include "PhysicsWorld.h"

#include "Collision/Shapes/Shape.h"
#include "Collision/Shapes/SphereShape.h"
#include "Collision/Shapes/BoxShape.h"
#include "Collision/Shapes/CapsuleShape.h"

#include "external/imgui/imgui.h"

#include "Dynamics/DistanceConstraint.h"
#include "Dynamics/ConstraintCoordinator.h"

void RopeSetting(vx::DistanceConstraint& constraint)
{
	constraint.mMinDistance = 1.25f;
	constraint.mMaxDistance = 2.5f;//6.0f;
	constraint.mSpring.tunningMode = vx::ESpringTuningMode::FrequencyDamping;
	//constraint.mSpring.stiffness = 0.0f;
	constraint.mSpring.frequency = vx::DegToRad(270.0f);

	constraint.mSpring.dampingRatio = 0.4f;


	//constraint.mSpring.tunningMode = vx::ESpringTuningMode::StiffnessSoftness;
	//constraint.mSpring.stiffness = 10000.0f;
	//constraint.mSpring.softness = 0.0f;// 1e-5f;
}

void SuspensionShockSettingCriticalDamping(vx::DistanceConstraint& constraint)
{
	///constraint.mMinDistance = -constraint.mRestLength;
	//constraint.mMaxDistance = constraint.mRestLength;
	constraint.mSpring.tunningMode = vx::ESpringTuningMode::FrequencyDamping;
	constraint.mSpring.frequency = vx::DegToRad(360.0f);
	constraint.mSpring.dampingRatio = 0.1f;
}

void SuspensionShockSetting(vx::DistanceConstraint& constraint)
{
	//constraint.mMinDistance = -constraint.mRestLength;
	//constraint.mMaxDistance = constraint.mRestLength;
	constraint.mSpring.tunningMode = vx::ESpringTuningMode::FrequencyDamping;
	constraint.mSpring.frequency = vx::DegToRad(270.0f);
	constraint.mSpring.dampingRatio = 2.0f;
}


void HardBarSetting(vx::DistanceConstraint& constraint)
{

	constraint.mMaxDistance = 2.5f;// constraint.mRestLength * 2.0f;
	constraint.mMinDistance = 2.5f;// constraint.mRestLength * 2.0f;
	constraint.mSpring.tunningMode = vx::ESpringTuningMode::StiffnessSoftness;
	//constraint.mSpring.tunningMode = vx::ESpringTuningMode::FrequencyDamping;
	constraint.mSpring.stiffness = 0.0f;
	constraint.mSpring.dampingRatio = 0.0f;
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

	vx::DistanceConstraint joint;
	joint.mBodyA = &mPhysicsWorld->GetBodies()[0];
	joint.mBodyB = &mPhysicsWorld->GetBodies()[1];
	joint.mLocalAnchorA = vx::Vec3(0.0f, 0.5f, 0.0f);
	joint.mLocalAnchorB = vx::Vec3(0.0f, 0.5f, 0.0f);

	//hard bar 
	RopeSetting(joint);

	//mNotInPipelineJoints.reserve(25);
	auto& phys_constraint_coord = mPhysicsWorld->mConstraintCoordinator;
	phys_constraint_coord->AddConstraintT(joint);

	vx::CapsuleShape* unit_capsule = new vx::CapsuleShape(0.5f, 0.5f);
	dyn_bodies_settings.position = vx::Vec3(-2.0f, 5.5f, 0.0f);
	dyn_bodies_settings.debug_name = "capsule";
	dyn_bodies_settings.shape = unit_capsule;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	//mNotInPipelineJoints.push_back(mJoint);
	vx::DistanceConstraint* new_j = phys_constraint_coord->AddConstraintT(joint);
	//auto& new_joint = mNotInPipelineJoints.back();
	new_j->mBodyA = &mPhysicsWorld->GetBodies()[1];
	new_j->mBodyB = &mPhysicsWorld->GetBodies()[2];
	//lets test placing the achor to the side of the capsule, instead of the origin
	new_j->mLocalAnchorA = Vec3(0.0f, -0.5f, 0.0f);
	new_j->mLocalAnchorB = Vec3(0.0f, 1.0f, 0.0f);
	RopeSetting(*new_j);

	dyn_bodies_settings.position = vx::Vec3(-2.0f, 5.5f, 0.0f);
	dyn_bodies_settings.debug_name = "box";
	dyn_bodies_settings.shape = unit_box;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	//mNotInPipelineJoints.push_back(mJoint);
	//auto& new_joint2 = mNotInPipelineJoints.back();
	auto& new_joint2 = *phys_constraint_coord->AddConstraintT(joint);
	new_joint2.mLocalAnchorA = Vec3(0.0f, -1.0f, 0.0f); //quick offset
	new_joint2.mLocalAnchorB = Vec3(0.5f); //quick offset
	new_joint2.mBodyA = &mPhysicsWorld->GetBodies()[2];
	new_joint2.mBodyB = &mPhysicsWorld->GetBodies()[3];
	RopeSetting(new_joint2);


	vx::DistanceConstraint constraint;
	HardBarSetting(constraint);

	dyn_bodies_settings.position = vx::Vec3(0.0f, 3.5f, 0.0f);
	dyn_bodies_settings.debug_name = "box";
	dyn_bodies_settings.shape = unit_box;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(0.0f, 0.5f, 0.0f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(3.5f, 0.5f, 0.0f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(3.5f, 3.5f, 0.0f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	vx::Body* body_a = &mPhysicsWorld->GetBodies()[4];
	vx::Body* body_b = &mPhysicsWorld->GetBodies()[5];
	vx::Body* body_c = &mPhysicsWorld->GetBodies()[6];
	vx::Body* body_d = &mPhysicsWorld->GetBodies()[7];

	//vx::DistanceConstraint& c0 = mNotInPipelineJoints.emplace_back(constraint);
	//vx::DistanceConstraint& c1 = mNotInPipelineJoints.emplace_back(constraint);
	//vx::DistanceConstraint& c2 = mNotInPipelineJoints.emplace_back(constraint);
	//vx::DistanceConstraint& c3 = mNotInPipelineJoints.emplace_back(constraint);

	//vx::DistanceConstraint& c4 = mNotInPipelineJoints.emplace_back(constraint);
	//vx::DistanceConstraint& c5 = mNotInPipelineJoints.emplace_back(constraint);


	vx::DistanceConstraint& c0 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& c1 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& c2 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& c3 = *phys_constraint_coord->AddConstraintT(constraint);

	vx::DistanceConstraint& c4 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& c5 = *phys_constraint_coord->AddConstraintT(constraint);

	c0.mBodyA = body_a;
	c0.mBodyB = body_b;
	//c0.mLocalAnchorA = vx::Vec3(0.0f, -0.5f, 0.0f);
	//c0.mLocalAnchorB = vx::Vec3(0.0f, 0.5f, 0.0f);

	c1.mBodyA = body_b;
	c1.mBodyB = body_c;
	//c1.mLocalAnchorA = vx::Vec3(0.5f, 0.0f, 0.0f);
	//c1.mLocalAnchorB = vx::Vec3(-0.5f, 0.0f, 0.0f);

	c2.mBodyA = body_c;
	c2.mBodyB = body_d;
	//c2.mLocalAnchorA = vx::Vec3(0.0f, 0.5f, 0.0f);
	//c2.mLocalAnchorB = vx::Vec3(0.0f, -0.5f, 0.0f);

	c3.mBodyA = body_d;
	c3.mBodyB = body_a;
	//c3.mLocalAnchorA = vx::Vec3(-0.5f, 0.0f, 0.0f);
	//c3.mLocalAnchorB = vx::Vec3(0.5f, 0.0f, 0.0f);


	c4.mBodyA = body_a;
	c4.mBodyB = body_c;
	//c4.mLocalAnchorA = vx::Vec3(0.5f, -0.5f, 0.0f);
	//c4.mLocalAnchorB = vx::Vec3(-0.5f, 0.5f, 0.0f);


	c5.mBodyA = body_b;
	c5.mBodyB = body_d;
	//c5.mLocalAnchorA = vx::Vec3(0.5f, 0.5f, 0.0f);
	//c5.mLocalAnchorB = vx::Vec3(-0.5f, -0.5f, 0.0f);

	//achor to the static 
	vx::BodySettings static_bodies_settings = vx::BodySettings::DefaultStaticConstruct();
	static_bodies_settings.position = vx::Vec3(0.0f, 12.5f, 0.0f);
	static_bodies_settings.debug_name = "box";
	static_bodies_settings.shape = unit_box;
	static_bodies_settings.intialVelocity = vx::Vec3(1.0f);
	uint32 static_body_idx = mPhysicsWorld->GetBodies().size();
	mPhysicsWorld->CreateBody(static_bodies_settings);
	constraint = vx::DistanceConstraint();
	RopeSetting(constraint);
	constraint.mMinDistance = 0.75f;
	constraint.mMaxDistance = 1.5f;
	constraint.mBodyA = &mPhysicsWorld->GetBodies()[static_body_idx];
	constraint.mLocalAnchorA = Vec3(0.0f, -0.5f, 0.0f);

	dyn_bodies_settings.position = vx::Vec3(0.0f, 12.5f, 0.0f);
	dyn_bodies_settings.shape = new vx::CapsuleShape(0.5f, 0.5f);;
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	constraint.mBodyB = &mPhysicsWorld->GetBodies().back();// body_d;
	constraint.mLocalAnchorB = Vec3(0.0f, 1.0f, 0.0f);
	phys_constraint_coord->AddConstraintT(constraint);

	//quickk reverse, B already set
	constraint.mLocalAnchorB = Vec3(0.0f, -1.0f, 0.0f);
	constraint.mBodyA = body_d;
	constraint.mLocalAnchorA = Vec3(0.0f, 0.5f, 0.0f);
	phys_constraint_coord->AddConstraintT(constraint);


	CreateLattice();

	/// Ground plane
	CreateGroundPlane(100.0f);
}

void JointScenario::PostPhysicsStep(float dt)
{
	//for(auto& joint : mNotInPipelineJoints)
	//	joint.Solve(dt);

	//if (mTrackCapsuleSphereRope)
	//{
	//	vx::Body* _a = mTrackCapsuleSphereRope->mBodyA;
	//	vx::Body* _b = mTrackCapsuleSphereRope->mBodyB;

	//	if (_a && _b)
	//	{
	//		vx::Vec3 rA = mTrackCapsuleSphereRope->mLocalAnchorA;
	//		vx::Vec3 rAw = _a->GetOrientation().Rotate(rA) + _a->GetPosition();
	//		vx::Vec3 dir = (rAw - _b->GetPosition()).Normalised();
	//		vx::Vec3 rB = dir * _b->GetShape()->GetHalfExtents();
	//		mTrackCapsuleSphereRope->mLocalAnchorB = rB;

	//		mTrackCapsuleSphereRope->DrawConstraintBounds(mDebugGizmos, rAw, rB + _b->GetPosition());
	//	}

	//}

	Scenario::PostPhysicsStep(dt);

	//if (mDebugGizmos)
	//{
	//	for (const auto& joint : mNotInPipelineJoints)
	//		joint.DebugGizmos(mDebugGizmos);
	//}
}


//Constraint window
void ConstaintPanel(DistanceConstraint& constraint)
{
	if(constraint.mBodyA && constraint.mBodyB)
		ImGui::Text("Body A ID: %d \nBody B ID: %d", constraint.mBodyA->GetID(), constraint.mBodyB->GetID());

	ImGui::DragFloat3("Local Anchor A", &constraint.mLocalAnchorA[0], 0.01f);
	ImGui::DragFloat3("Local Anchor B", &constraint.mLocalAnchorB[0], 0.01f);
	
	ImGui::DragFloat("Min Distance", &constraint.mMinDistance, 0.1f);
	ImGui::DragFloat("Max Distance", &constraint.mMaxDistance, 0.1f);

	ImGui::Text("Accumulated Lambda: %f", constraint.mAccumulatedLambda);
	
	ImGui::SeparatorText("Spring Setting");
	auto& spring = constraint.mSpring;

	EditorImGui::Combo("Tuning Mode", spring.tunningMode, "Stiffness Softness\0""Frequency Damping\0""\0");
	if (spring.tunningMode == ESpringTuningMode::StiffnessSoftness)
		ImGui::DragFloat("Stiffness [N/m]", &spring.stiffness);
	else
		ImGui::SliderAngle("Frequency [Hz:Rad/sec]", &spring.frequency, 0.0f);
	ImGui::DragFloat("Damping Ratio", &spring.dampingRatio, 0.01f);
	ImGui::SliderFloat("Softness", &spring.softness, 0.0f, 1.0f);
}


void JointScenario::OnUI()
{
	Scenario::OnUI();

	if (mPhysicsWorld == nullptr)
		return;

	if (ImGui::Begin("Joint Window"))
	{
		static bool use_new = true;

		//if (use_new)
		//	ConstaintPanel(mJoint);
		//else
		//{
		//	ImGui::SliderFloat("Joint Rest length", &mJoint.mRestLength, 0.0f, 10.0f);
		//	ImGui::SliderFloat("Joint Damping Ratio", &mJoint.mDampingRatio, 0.0f, 1.0f);
		//	ImGui::DragFloat("Joint Stiffness", &mJoint.mStiffness);
		//}

		int _idx = 1;
		for (auto& joint : mPhysicsWorld->mConstraintCoordinator->GetConstraints())
		{
			ImGui::Separator();
			ImGui::Spacing();
			ImGui::PushID(&joint);
			ImGui::Text("joint %d", _idx++);
			if (use_new)
				ConstaintPanel(*static_cast<vx::DistanceConstraint*>(joint));
				//ConstaintPanel(joint);
			else
			{
				//ImGui::SliderFloat("Joint Rest length", &joint.mRestLength, 0.0f, 10.0f);
				//ImGui::SliderFloat("Joint Damping Ratio", &joint.mDampingRatio, 0.0f, 1.0f);
				//ImGui::DragFloat("Joint Stiffness", &joint.mStiffness);
			}
			ImGui::PopID();
		}
	}
	ImGui::End();
}

void JointScenario::CreateLattice()
{

	auto& phys_constraint_coord = mPhysicsWorld->mConstraintCoordinator;



	float depth = -4.0f;
	vx::uint32 first_body = mPhysicsWorld->GetBodies().size();

	vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();

	dyn_bodies_settings.position = vx::Vec3(0.0f);
	dyn_bodies_settings.debug_name = "capsule"; //"box";
	dyn_bodies_settings.shape = new vx::CapsuleShape(0.5f, 0.5f);//vx::BoxShape(0.5f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	dyn_bodies_settings.position = vx::Vec3(0.0f, 3.5f, depth);
	dyn_bodies_settings.debug_name = "sphere"; //"box";
	dyn_bodies_settings.shape = new vx::SphereShape(0.5f);//vx::BoxShape(0.5f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(0.0f, 0.5f, depth);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(3.5f, 0.5f, depth);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(3.5f, 3.5f, depth);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);

	depth -= 2.0f;

	dyn_bodies_settings.position = vx::Vec3(0.0f, 3.5f, depth);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(0.0f, 0.5f, depth);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(3.5f, 0.5f, depth);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);
	dyn_bodies_settings.position = vx::Vec3(3.5f, 3.5f, depth);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);


	vx::Body* capsule0 = &mPhysicsWorld->GetBodies()[first_body++];
	vx::Body* capsule1 = &mPhysicsWorld->GetBodies()[first_body++];
	vx::Body* body_a = &mPhysicsWorld->GetBodies()[first_body++];
	vx::Body* body_b = &mPhysicsWorld->GetBodies()[first_body++];
	vx::Body* body_c = &mPhysicsWorld->GetBodies()[first_body++];
	vx::Body* body_d = &mPhysicsWorld->GetBodies()[first_body++];
	vx::Body* body_a1 = &mPhysicsWorld->GetBodies()[first_body++];
	vx::Body* body_b1 = &mPhysicsWorld->GetBodies()[first_body++];
	vx::Body* body_c1 = &mPhysicsWorld->GetBodies()[first_body++];
	vx::Body* body_d1 = &mPhysicsWorld->GetBodies()[first_body++];

	vx::DistanceConstraint constraint;
	RopeSetting(constraint);

	constraint.mBodyA = capsule0;
	constraint.mLocalAnchorA = vx::Vec3(0.0f, -1.0f, 0.0f);
	constraint.mBodyB = capsule1;
	phys_constraint_coord->AddConstraintT(constraint);
	constraint.mMinDistance = 3.0f;
	constraint.mMaxDistance = 6.0f;
	constraint.mBodyA = capsule1;
	constraint.mLocalAnchorA = vx::Vec3(0.0f, 1.0f, 0.0f);
	constraint.mBodyB = body_a;
	constraint.mLocalAnchorB = vx::Vec3(0.25f);
	mTrackCapsuleSphereRope = phys_constraint_coord->AddConstraintT(constraint);
	constraint.mLocalAnchorA = vx::Vec3(0.0f, -1.0f, 0.0f);
	constraint.mBodyB = body_d;
	phys_constraint_coord->AddConstraintT(constraint);
	
	constraint = vx::DistanceConstraint();

	//SuspensionShockSetting(constraint);
	HardBarSetting(constraint);
	constraint.mMaxDistance = 3.5f;// constraint.mRestLength * 2.0f;
	constraint.mMinDistance = 3.5f;// constraint.mRestLength * 2.0f;
	constraint.mSpring.tunningMode = vx::ESpringTuningMode::StiffnessSoftness;
	constraint.mSpring.stiffness = 0.0f;
	constraint.mSpring.dampingRatio = 0.0f;

	vx::DistanceConstraint& c0 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& c1 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& c2 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& c3 = *phys_constraint_coord->AddConstraintT(constraint);

	vx::DistanceConstraint& c4 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& c5 = *phys_constraint_coord->AddConstraintT(constraint);

	vx::DistanceConstraint& c01 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& c11 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& c21 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& c31 = *phys_constraint_coord->AddConstraintT(constraint);
							  
	vx::DistanceConstraint& c41 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& c51 = *phys_constraint_coord->AddConstraintT(constraint);

	c0.mBodyA = body_a;
	c0.mBodyB = body_b;
	//c0.mLocalAnchorA = vx::Vec3(0.0f, -0.5f, 0.0f);
	//c0.mLocalAnchorB = vx::Vec3(0.0f, 0.5f, 0.0f);

	c1.mBodyA = body_b;
	c1.mBodyB = body_c;
	//c1.mLocalAnchorA = vx::Vec3(0.5f, 0.0f, 0.0f);
	//c1.mLocalAnchorB = vx::Vec3(-0.5f, 0.0f, 0.0f);

	c2.mBodyA = body_c;
	c2.mBodyB = body_d;
	//c2.mLocalAnchorA = vx::Vec3(0.0f, 0.5f, 0.0f);
	//c2.mLocalAnchorB = vx::Vec3(0.0f, -0.5f, 0.0f);

	c3.mBodyA = body_d;
	c3.mBodyB = body_a;
	//c3.mLocalAnchorA = vx::Vec3(-0.5f, 0.0f, 0.0f);
	//c3.mLocalAnchorB = vx::Vec3(0.5f, 0.0f, 0.0f);


	c4.mBodyA = body_a;
	c4.mBodyB = body_c;
	//c4.mLocalAnchorA = vx::Vec3(0.5f, -0.5f, 0.0f);
	//c4.mLocalAnchorB = vx::Vec3(-0.5f, 0.5f, 0.0f);


	c5.mBodyA = body_b;
	c5.mBodyB = body_d;
	//c5.mLocalAnchorA = vx::Vec3(0.5f, 0.5f, 0.0f);
	//c5.mLocalAnchorB = vx::Vec3(-0.5f, -0.5f, 0.0f);




	c01.mBodyA = body_a1;
	c01.mBodyB = body_b1;
	//c0.mLocalAnchorA = vx::Vec3(0.0f, -0.5f, 0.0f);
	//c0.mLocalAnchorB = vx::Vec3(0.0f, 0.5f, 0.0f);

	c11.mBodyA = body_b1;
	c11.mBodyB = body_c1;
	//c1.mLocalAnchorA = vx::Vec3(0.5f, 0.0f, 0.0f);
	//c1.mLocalAnchorB = vx::Vec3(-0.5f, 0.0f, 0.0f);

	c21.mBodyA = body_c1;
	c21.mBodyB = body_d1;
	//c2.mLocalAnchorA = vx::Vec3(0.0f, 0.5f, 0.0f);
	//c2.mLocalAnchorB = vx::Vec3(0.0f, -0.5f, 0.0f);

	c31.mBodyA = body_d1;
	c31.mBodyB = body_a1;
	//c3.mLocalAnchorA = vx::Vec3(-0.5f, 0.0f, 0.0f);
	//c3.mLocalAnchorB = vx::Vec3(0.5f, 0.0f, 0.0f);


	c41.mBodyA = body_a1;
	c41.mBodyB = body_c1;
	//c4.mLocalAnchorA = vx::Vec3(0.5f, -0.5f, 0.0f);
	//c4.mLocalAnchorB = vx::Vec3(-0.5f, 0.5f, 0.0f);


	c51.mBodyA = body_b1;
	c51.mBodyB = body_d1;
	//c5.mLocalAnchorA = vx::Vec3(0.5f, 0.5f, 0.0f);
	//c5.mLocalAnchorB = vx::Vec3(-0.5f, -0.5f, 0.0f);



	vx::DistanceConstraint& a_c_a1 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& b_c_b1 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& c_c_c1 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& d_c_d1 = *phys_constraint_coord->AddConstraintT(constraint);

	a_c_a1.mBodyA = body_a;
	a_c_a1.mBodyB = body_a1;

	b_c_b1.mBodyA = body_b;
	b_c_b1.mBodyB = body_b1;

	c_c_c1.mBodyA = body_c;
	c_c_c1.mBodyB = body_c1;

	d_c_d1.mBodyA = body_d;
	d_c_d1.mBodyB = body_d1;


	vx::DistanceConstraint& c_c_d1 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& d_c_c1 = *phys_constraint_coord->AddConstraintT(constraint);

	vx::DistanceConstraint& a_c_b1 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& b_c_a1 = *phys_constraint_coord->AddConstraintT(constraint);


	c_c_d1.mBodyA = body_c;
	c_c_d1.mBodyB = body_d1;

	d_c_c1.mBodyA = body_d;
	d_c_c1.mBodyB = body_c1;

	a_c_b1.mBodyA = body_a;
	a_c_b1.mBodyB = body_b1;

	b_c_a1.mBodyA = body_b;
	b_c_a1.mBodyB = body_a1;


	vx::DistanceConstraint& a_c_d1 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& d_c_a1 = *phys_constraint_coord->AddConstraintT(constraint);

	vx::DistanceConstraint& b_c_c1 = *phys_constraint_coord->AddConstraintT(constraint);
	vx::DistanceConstraint& c_c_b1 = *phys_constraint_coord->AddConstraintT(constraint);


	a_c_d1.mBodyA = body_a;
	a_c_d1.mBodyB = body_d1;

	d_c_a1.mBodyA = body_d;
	d_c_a1.mBodyB = body_a1;

	b_c_c1.mBodyA = body_b;
	b_c_c1.mBodyB = body_c1;

	c_c_b1.mBodyA = body_c;
	c_c_b1.mBodyB = body_b1;
}
