#include "RagdollBuilder.h"

#include "Vortrix/Collision/Shapes/BoxShape.h"
#include "Vortrix/Collision/Shapes/CapsuleShape.h"

#include "Vortrix/PhysicsWorld.h"

#include "Constraints/PointConstraint.h"

namespace vx{

	void Ragdoll::AddBodyPart(const BodyID id, const char* name)
	{
		mBodyIDs.push_back(id);
		mBodyNames.push_back({ int(mBodyIDs.size() - 1), StackString<20>(name) });
	}
	void Ragdoll::AddConstraint(Constraint* constraint, const char* name)
	{
		mConstraints.push_back(constraint);
		mConstraintNames.push_back({ int(mConstraints.size() - 1), StackString<20>(name) });
	}



	void RagdollBuilder::GenerateBoxSingleTorsoShapeParts()
	{
		VX_ASSERT(!HasLimbShape(ERagdollLimbsShape::Box_Head));
		//vx::BoxShapeSettings head_settings(0.22f);
		//vx::BoxShapeSettings torso_settings(Vec3(0.35f, 0.5f, 0.18f));
		//vx::BoxShapeSettings upper_arm_settings(Vec3(0.1f, 0.45f, 0.1f));
		//vx::BoxShapeSettings lower_arm_settings(Vec3(0.09f, 0.45f, 0.09f));
		//vx::BoxShapeSettings upper_leg_settings(Vec3(0.12f, 0.65f, 0.12f));
		//vx::BoxShapeSettings lower_leg_settings(Vec3(0.1f, 0.65f, 0.1f));

		vx::BoxShapeSettings head_settings(0.17f);
		vx::BoxShapeSettings torso_settings(Vec3(0.3f, 0.25f, 0.1f));
		vx::BoxShapeSettings upper_arm_settings(Vec3(0.06f, 0.21f, 0.06f));
		vx::BoxShapeSettings lower_arm_settings(Vec3(0.05f, 0.20f, 0.05f));
		vx::BoxShapeSettings upper_leg_settings(Vec3(0.075f, 0.275f, 0.075f));
		vx::BoxShapeSettings lower_leg_settings(Vec3(0.06f, 0.26f, 0.06f));

		//head_settings.mHalfExtent * 2.72f;
		//torso_settings.mHalfExtent * 2.72f;
		//upper_arm_settings.mHalfExtent * 2.72f;
		//lower_arm_settings.mHalfExtent * 2.72f;
		//upper_leg_settings.mHalfExtent * 2.72f;
		//lower_leg_settings.mHalfExtent * 2.72f;

		///this for now before mass ratio, distribution

		float each_density = 12000 * 5.0f;

		head_settings.SetDensity(each_density);
		torso_settings.SetDensity(each_density);
		upper_arm_settings.SetDensity(each_density);
		lower_arm_settings.SetDensity(each_density);
		upper_leg_settings.SetDensity(each_density);
		lower_leg_settings.SetDensity(each_density);

		RefConst<Shape> head = vx::MakeRef<vx::BoxShape>(head_settings);
		RefConst<Shape> torso = vx::MakeRef<vx::BoxShape>(torso_settings);
		RefConst<Shape> upper_arm = vx::MakeRef<vx::BoxShape>(upper_arm_settings);
		RefConst<Shape> lower_arm = vx::MakeRef<vx::BoxShape>(lower_arm_settings);
		RefConst<Shape> upper_leg = vx::MakeRef<vx::BoxShape>(upper_leg_settings);
		RefConst<Shape> lower_leg = vx::MakeRef<vx::BoxShape>(lower_leg_settings);

		mCacheLimbShapes[size_t(ERagdollLimbsShape::Box_Head)] = head;
		mCacheLimbShapes[size_t(ERagdollLimbsShape::Box_Torso)] = torso;
		mCacheLimbShapes[size_t(ERagdollLimbsShape::Box_Left_UpperArm)] = upper_arm;
		mCacheLimbShapes[size_t(ERagdollLimbsShape::Box_Left_LowerArm)] = lower_arm;
		mCacheLimbShapes[size_t(ERagdollLimbsShape::Box_Left_UpperLeg)] = upper_leg;
		mCacheLimbShapes[size_t(ERagdollLimbsShape::Box_Left_LowerLeg)] = lower_leg;
	}

	void RagdollBuilder::GenerateBoxChestPelvis()
	{
		VX_ASSERT(!HasLimbShape(ERagdollLimbsShape::Box_Chest));
		//vx::BoxShapeSettings chest_settings(Vec3(0.38f, 0.35f, 0.20f));
		//vx::BoxShapeSettings pelvis_settings(Vec3(0.30f, 0.20f, 0.18f));

		vx::BoxShapeSettings chest_settings(Vec3(0.25f, 0.2f, 0.10f));
		vx::BoxShapeSettings pelvis_settings(Vec3(0.25f, 0.10f, 0.1f));


		float each_density = 12000;// *5.0f;
		chest_settings.SetDensity(each_density);
		pelvis_settings.SetDensity(each_density);

		mCacheLimbShapes[size_t(ERagdollLimbsShape::Box_Chest)] = vx::MakeRef<vx::BoxShape>(chest_settings);
		mCacheLimbShapes[size_t(ERagdollLimbsShape::Box_Plevis)] = vx::MakeRef<vx::BoxShape>(pelvis_settings);
	}


	void RagdollBuilder::GenerateCapsuleSingleTorsoShapeParts()
	{
		VX_ASSERT(!HasLimbShape(ERagdollLimbsShape::Capsule_Head));

		vx::CapsuleShapeSettings head_settings(0.10f, 0.075f);
		//vx::CapsuleShapeSettings torso_settings(0.25f, 0.1f);
		vx::CapsuleShapeSettings torso_settings(0.15f, 0.15f);
		vx::CapsuleShapeSettings upper_arm_settings(0.096f, 0.15f);
		vx::CapsuleShapeSettings lower_arm_settings(0.095f, 0.15f);
		vx::CapsuleShapeSettings upper_leg_settings(0.105f, 0.2f);
		//vx::CapsuleShapeSettings upper_leg_settings(0.075f, 0.2f);
		vx::CapsuleShapeSettings lower_leg_settings(0.096f, 0.2f);

		///this for now before mass ratio, distribution

		float each_density = 12000 * 9.0f;

		head_settings.SetDensity(each_density);
		torso_settings.SetDensity(each_density);
		upper_arm_settings.SetDensity(each_density);
		lower_arm_settings.SetDensity(each_density);
		upper_leg_settings.SetDensity(each_density);
		lower_leg_settings.SetDensity(each_density);

		mCacheLimbShapes[size_t(ERagdollLimbsShape::Capsule_Head)] = vx::MakeRef<vx::CapsuleShape>(head_settings);
		mCacheLimbShapes[size_t(ERagdollLimbsShape::Capsule_Torso)] = vx::MakeRef<vx::CapsuleShape>(torso_settings);
		mCacheLimbShapes[size_t(ERagdollLimbsShape::Capsule_Left_UpperArm)] = vx::MakeRef<vx::CapsuleShape>(upper_arm_settings);
		mCacheLimbShapes[size_t(ERagdollLimbsShape::Capsule_Left_LowerArm)] = vx::MakeRef<vx::CapsuleShape>(lower_arm_settings);
		mCacheLimbShapes[size_t(ERagdollLimbsShape::Capsule_Left_UpperLeg)] = vx::MakeRef<vx::CapsuleShape>(upper_leg_settings);
		mCacheLimbShapes[size_t(ERagdollLimbsShape::Capsule_Left_LowerLeg)] = vx::MakeRef<vx::CapsuleShape>(lower_leg_settings);
	}

	void RagdollBuilder::BuildBoxLimbsSingleTorso(const RagdollSettings& settings, Ragdoll* ragdoll)
	{
		if (!HasLimbShape(ERagdollLimbsShape::Complete_Box_Set))
			GenerateBoxSingleTorsoShapeParts();


		RefConst<Shape> head = GetLimbShapePart(ERagdollLimbsShape::Box_Head);
		RefConst<Shape> torso_shape = GetLimbShapePart(ERagdollLimbsShape::Box_Torso);
		RefConst<Shape> upper_arm = GetLimbShapePart(ERagdollLimbsShape::Box_Left_UpperArm);
		RefConst<Shape> lower_arm = GetLimbShapePart(ERagdollLimbsShape::Box_Left_LowerArm);
		RefConst<Shape> upper_leg = GetLimbShapePart(ERagdollLimbsShape::Box_Left_UpperLeg);
		RefConst<Shape> lower_leg = GetLimbShapePart(ERagdollLimbsShape::Box_Left_LowerLeg);

		Vec3 torso_half_size = torso_shape->HalfExtents();

		float limb_offset = settings.limbsOffset;

		float upper_arm_thickness = upper_arm->HalfExtents().X();
		Vec3 head_pos(0.0f, head->HalfExtents().Y() + torso_shape->HalfExtents().Y() + limb_offset, 0.0f);
		float shoulder_x = torso_shape->HalfExtents().X();// +upper_arm_thickness;
		float shoulder_y = torso_shape->HalfExtents().Y() * 0.6f;

		//float upper_arm_length = upper_arm->HalfExtents().Y(); 
		float upper_arm_half_length = upper_arm->HalfExtents().Y();
		//float lower_arm_length = lower_arm->HalfExtents().Y() * 2.0f; 
		Vec3  upper_armLPos(-(shoulder_x + upper_arm_half_length + limb_offset), shoulder_y, 0.0f);
		Vec3  upper_armRPos(upper_armLPos * vx::Vec3(-1, 1, 1));

		Vec3 lower_armLPos = upper_armLPos - Vec3(upper_arm_half_length + lower_arm->HalfExtents().Y() + limb_offset, 0, 0);
		Vec3 lower_armRPos(lower_armLPos * vx::Vec3(-1, 1, 1));

		float torso_half_width = torso_half_size.X();
		float torso_half_height = torso_half_size.Y();
		float hipX = torso_half_width * 0.45f;
		float upper_leg_half_length = upper_leg->HalfExtents().Y();
		Vec3 upper_legRPos(hipX, -(torso_half_height + upper_leg_half_length + limb_offset), 0);
		Vec3 upper_legLPos(upper_legRPos * vx::Vec3(-1, 1, 1));


		float lower_leg_length = lower_leg->HalfExtents().Y();
		Vec3 lower_legRPos = upper_legRPos - Vec3(0, upper_leg_half_length + lower_leg_length + limb_offset, 0);
		Vec3 lower_legLPos(lower_legRPos * vx::Vec3(-1, 1, 1));


		std::vector<Body*> bodies;
		vx::BodySettings body_settings = vx::BodySettings::DefaultDynamicConstruct();


		body_settings.shape = head;
		body_settings.debug_name = "Head";
		body_settings.position = head_pos;
		body_settings.orientation = vx::Quat::Identity();
		Body* _head = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(_head);

		/// body
		body_settings.shape = torso_shape;
		body_settings.debug_name = "Torso";
		body_settings.position = vx::Vec3(0.0f, 0.0f, 0.0f);
		body_settings.orientation = vx::Quat::Identity();
		Body* torso = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(torso);

		/// upper left arm 
		body_settings.shape = upper_arm;
		body_settings.debug_name = "Left Upper Arm";
		body_settings.position = upper_armLPos;
		body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
		Body* upper_armL = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(upper_armL);

		/// upper right arm 
		body_settings.shape = upper_arm;
		body_settings.debug_name = "Right Upper Arm";
		body_settings.position = upper_armRPos;
		body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
		Body* upper_armR = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(upper_armR);

		/// lower left arm 
		body_settings.shape = lower_arm;
		body_settings.debug_name = "Left Lower Arm";
		body_settings.position = lower_armLPos;
		body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
		Body* lower_armL = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(lower_armL);

		/// lower right arm 
		body_settings.shape = lower_arm;
		body_settings.debug_name = "Right Lower Arm";
		body_settings.position = lower_armRPos;
		body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
		Body* lower_armR = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(lower_armR);



		/// upper left leg 
		body_settings.shape = upper_leg;
		body_settings.debug_name = "Left Upper Leg";
		body_settings.position = upper_legLPos;
		body_settings.orientation = vx::Quat::Identity();
		Body* upper_legL = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(upper_legL);

		/// upper right leg 
		body_settings.shape = upper_leg;
		body_settings.debug_name = "Right Upper Leg";
		body_settings.position = upper_legRPos;
		body_settings.orientation = vx::Quat::Identity();
		Body* upper_legR = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(upper_legR);


		/// lowe left leg 
		body_settings.shape = lower_leg;
		body_settings.debug_name = "Left Lower Leg";
		body_settings.position = lower_legLPos;
		body_settings.orientation = vx::Quat::Identity();
		Body* lower_legL = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(lower_legL);

		/// lower right leg 
		body_settings.shape = lower_leg;
		body_settings.debug_name = "Right Lower Leg";
		body_settings.position = lower_legRPos;
		body_settings.orientation = vx::Quat::Identity();
		Body* lower_legR = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(lower_legR);



		float half_limb_offset = limb_offset * 0.5f;

		vx::PointConstraint joints[] =
		{

			vx::PointConstraint(_head,
				torso,
				vx::PointConstraintSettings(vx::Vec3(0.0f, -(head->HalfExtents().Y() + half_limb_offset), 0.0f),
					vx::Vec3(0.0f, torso_half_height + half_limb_offset, 0.0f), vx::EConstraintFrame::Local)), ///neck_joint

					///upper left arm
			vx::PointConstraint(
				upper_armL,
				torso,
				vx::PointConstraintSettings(
					vx::Vec3(0.0f, -(upper_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
					vx::Vec3(-(torso_half_width + half_limb_offset), shoulder_y, 0.0f),
					vx::EConstraintFrame::Local)), ///left_shoulder_body_upper_left_arm

				///upper right arm
		vx::PointConstraint(
			upper_armR,
			torso,
				vx::PointConstraintSettings(
					vx::Vec3(0.0f, -(upper_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
					vx::Vec3(torso_half_width + half_limb_offset, shoulder_y, 0.0f),
					vx::EConstraintFrame::Local)), ///left_arm__upper_left_lower_left_arn

				///lower left arm
		vx::PointConstraint(
			upper_armL,
			lower_armL,
			vx::PointConstraintSettings(
				vx::Vec3(0.0f, (upper_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
				vx::Vec3(0.0f, -(lower_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
				vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm

				///lower right arm elbow
				vx::PointConstraint(
					upper_armR,
					lower_armR,
					vx::PointConstraintSettings(
				vx::Vec3(0.0f, (upper_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
					vx::Vec3(0.0f, -(lower_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
					vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm


				/// left hip
vx::PointConstraint(
	torso,
	upper_legL,
	vx::PointConstraintSettings(
		vx::Vec3(-hipX, -(torso_half_height + half_limb_offset), 0.0f),
		vx::Vec3(0.0f, upper_leg->HalfExtents().Y() + half_limb_offset, 0.0f),
		vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm


				/// right hip
vx::PointConstraint(
	torso,
					upper_legR,
					vx::PointConstraintSettings(
						vx::Vec3(hipX, -(torso_half_height + half_limb_offset), 0.0f),
						vx::Vec3(0.0f, upper_leg->HalfExtents().Y() + half_limb_offset, 0.0f),
						vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm


				/// left knee
vx::PointConstraint(
	upper_legL,
	lower_legL,
	vx::PointConstraintSettings(
		vx::Vec3(0.0f, -(upper_leg->HalfExtents().Y() + half_limb_offset), 0.0f),
		vx::Vec3(0.0f, (lower_leg->HalfExtents().Y() + half_limb_offset), 0.0f),
		vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm

				/// right knee
vx::PointConstraint(
	upper_legR,
	lower_legR,
	vx::PointConstraintSettings(
		vx::Vec3(0.0f, -(upper_leg->HalfExtents().Y() + half_limb_offset), 0.0f),
		vx::Vec3(0.0f, (lower_leg->HalfExtents().Y() + half_limb_offset), 0.0f),
		vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm

		};
		//move body upward world frame
		for (auto& b : bodies)
			b->SetPosition(b->Position() + settings.position);

		uint32 count = mPhysicsWorld->GetConstraints().size();
		mPhysicsWorld->CreateConstraintsT(joints, 9);

		if(ragdoll)
		{
			ragdoll->AddBodyPart(lower_legR->GetID(), "Right Upper Leg");
			ragdoll->AddBodyPart(lower_legL->GetID(), "Left Upper Leg");
			ragdoll->AddBodyPart(upper_legR->GetID(), "Right Upper Leg");
			ragdoll->AddBodyPart(upper_legL->GetID(), "Left Upper Leg");
			ragdoll->AddBodyPart(lower_armR->GetID(), "Right Lower Arm");
			ragdoll->AddBodyPart(upper_armL->GetID(), "Left Upper Arm");
			ragdoll->AddBodyPart(_head->GetID(), "Head");
			ragdoll->AddBodyPart(torso->GetID(), "Torso");
			ragdoll->AddBodyPart(upper_armR->GetID(), "Right Upper Arm");
			ragdoll->AddBodyPart(lower_armL->GetID(), "Left Lower Arm");

			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 0], "Neck");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 1], "Left Shoulder");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 2], "Right Shoulder");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 3], "Left Elbow");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 4], "Right Elbow");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 5], "Left Hip");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 6], "Right Hip");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 7], "Left Knee");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 8], "Right Knee");
		}
	}


	void RagdollBuilder::BuildCapsuleLimbsSingleTorso(const RagdollSettings& settings, Ragdoll* ragdoll)
	{
		if (!HasLimbShape(ERagdollLimbsShape::Complete_Capsule_Set))
			GenerateCapsuleSingleTorsoShapeParts();


		RefConst<Shape> head = GetLimbShapePart(ERagdollLimbsShape::Capsule_Head);
		RefConst<Shape> torso_shape = GetLimbShapePart(ERagdollLimbsShape::Capsule_Torso);
		RefConst<Shape> upper_arm = GetLimbShapePart(ERagdollLimbsShape::Capsule_Left_UpperArm);
		RefConst<Shape> lower_arm = GetLimbShapePart(ERagdollLimbsShape::Capsule_Left_LowerArm);
		RefConst<Shape> upper_leg = GetLimbShapePart(ERagdollLimbsShape::Capsule_Left_UpperLeg);
		RefConst<Shape> lower_leg = GetLimbShapePart(ERagdollLimbsShape::Capsule_Left_LowerLeg);

		Vec3 torso_half_size = torso_shape->HalfExtents();

		float limb_offset = settings.limbsOffset;

		float upper_arm_thickness = upper_arm->HalfExtents().X();
		Vec3 head_pos(0.0f, head->HalfExtents().Y() + torso_shape->HalfExtents().Y() + limb_offset, 0.0f);
		float shoulder_x = torso_shape->HalfExtents().X();// +upper_arm_thickness;
		float shoulder_y = torso_shape->HalfExtents().Y() * 0.6f;

		//float upper_arm_length = upper_arm->HalfExtents().Y(); 
		float upper_arm_half_length = upper_arm->HalfExtents().Y();
		//float lower_arm_length = lower_arm->HalfExtents().Y() * 2.0f; 
		Vec3  upper_armLPos(-(shoulder_x + upper_arm_half_length + limb_offset), shoulder_y, 0.0f);
		Vec3  upper_armRPos(upper_armLPos * vx::Vec3(-1, 1, 1));

		Vec3 lower_armLPos = upper_armLPos - Vec3(upper_arm_half_length + lower_arm->HalfExtents().Y() + limb_offset, 0, 0);
		Vec3 lower_armRPos(lower_armLPos * vx::Vec3(-1, 1, 1));

		float torso_half_width = torso_half_size.X();
		float torso_half_height = torso_half_size.Y();
		float hipX = torso_half_width * 0.45f;
		float upper_leg_half_length = upper_leg->HalfExtents().Y();
		Vec3 upper_legRPos(hipX, -(torso_half_height + upper_leg_half_length + limb_offset), 0);
		Vec3 upper_legLPos(upper_legRPos * vx::Vec3(-1, 1, 1));


		float lower_leg_length = lower_leg->HalfExtents().Y();
		Vec3 lower_legRPos = upper_legRPos - Vec3(0, upper_leg_half_length + lower_leg_length + limb_offset, 0);
		Vec3 lower_legLPos(lower_legRPos * vx::Vec3(-1, 1, 1));


		std::vector<Body*> bodies;
		vx::BodySettings body_settings = vx::BodySettings::DefaultDynamicConstruct();


		body_settings.shape = head;
		body_settings.debug_name = "Head";
		body_settings.position = head_pos;
		body_settings.orientation = vx::Quat::Identity();
		Body* _head = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(_head);

		/// body
		body_settings.shape = torso_shape;
		body_settings.debug_name = "Torso";
		body_settings.position = vx::Vec3(0.0f, 0.0f, 0.0f);
		body_settings.orientation = vx::Quat::Identity();
		Body* torso = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(torso);

		/// upper left arm 
		body_settings.shape = upper_arm;
		body_settings.debug_name = "Left Upper Arm";
		body_settings.position = upper_armLPos;
		body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
		Body* upper_armL = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(upper_armL);

		/// upper right arm 
		body_settings.shape = upper_arm;
		body_settings.debug_name = "Right Upper Arm";
		body_settings.position = upper_armRPos;
		body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
		Body* upper_armR = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(upper_armR);

		/// lower left arm 
		body_settings.shape = lower_arm;
		body_settings.debug_name = "Left Lower Arm";
		body_settings.position = lower_armLPos;
		body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
		Body* lower_armL = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(lower_armL);

		/// lower right arm 
		body_settings.shape = lower_arm;
		body_settings.debug_name = "Right Lower Arm";
		body_settings.position = lower_armRPos;
		body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
		Body* lower_armR = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(lower_armR);



		/// upper left leg 
		body_settings.shape = upper_leg;
		body_settings.debug_name = "Left Upper Leg";
		body_settings.position = upper_legLPos;
		body_settings.orientation = vx::Quat::Identity();
		Body* upper_legL = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(upper_legL);

		/// upper right leg 
		body_settings.shape = upper_leg;
		body_settings.debug_name = "Right Upper Leg";
		body_settings.position = upper_legRPos;
		body_settings.orientation = vx::Quat::Identity();
		Body* upper_legR = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(upper_legR);


		/// lowe left leg 
		body_settings.shape = lower_leg;
		body_settings.debug_name = "Left Lower Leg";
		body_settings.position = lower_legLPos;
		body_settings.orientation = vx::Quat::Identity();
		Body* lower_legL = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(lower_legL);

		/// lower right leg 
		body_settings.shape = lower_leg;
		body_settings.debug_name = "Right Lower Leg";
		body_settings.position = lower_legRPos;
		body_settings.orientation = vx::Quat::Identity();
		Body* lower_legR = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(lower_legR);


		float half_limb_offset = limb_offset * 0.5f;

		vx::PointConstraint joints[] =
		{

			vx::PointConstraint(_head,
				torso,
				vx::PointConstraintSettings(vx::Vec3(0.0f, -(head->HalfExtents().Y() + half_limb_offset), 0.0f),
					vx::Vec3(0.0f, torso_half_height + half_limb_offset, 0.0f), vx::EConstraintFrame::Local)), ///neck_joint

					///upper left arm
			vx::PointConstraint(
				upper_armL,
				torso,
				vx::PointConstraintSettings(
					vx::Vec3(0.0f, -(upper_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
					vx::Vec3(-(torso_half_width + half_limb_offset), shoulder_y, 0.0f),
					vx::EConstraintFrame::Local)), ///left_shoulder_body_upper_left_arm

				///upper right arm
		vx::PointConstraint(
			upper_armR,
			torso,
				vx::PointConstraintSettings(
					vx::Vec3(0.0f, -(upper_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
					vx::Vec3(torso_half_width + half_limb_offset, shoulder_y, 0.0f),
					vx::EConstraintFrame::Local)), ///left_arm__upper_left_lower_left_arn

				///lower left arm
		vx::PointConstraint(
			upper_armL,
			lower_armL,
			vx::PointConstraintSettings(
				vx::Vec3(0.0f, (upper_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
				vx::Vec3(0.0f, -(lower_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
				vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm

				///lower right arm elbow
				vx::PointConstraint(
					upper_armR,
					lower_armR,
					vx::PointConstraintSettings(
				vx::Vec3(0.0f, (upper_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
					vx::Vec3(0.0f, -(lower_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
					vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm


				/// left hip
vx::PointConstraint(
	torso,
	upper_legL,
	vx::PointConstraintSettings(
		vx::Vec3(-hipX, -(torso_half_height + half_limb_offset), 0.0f),
		vx::Vec3(0.0f, upper_leg->HalfExtents().Y() + half_limb_offset, 0.0f),
		vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm


				/// right hip
vx::PointConstraint(
	torso,
					upper_legR,
					vx::PointConstraintSettings(
						vx::Vec3(hipX, -(torso_half_height + half_limb_offset), 0.0f),
						vx::Vec3(0.0f, upper_leg->HalfExtents().Y() + half_limb_offset, 0.0f),
						vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm


				/// left knee
vx::PointConstraint(
	upper_legL,
	lower_legL,
	vx::PointConstraintSettings(
		vx::Vec3(0.0f, -(upper_leg->HalfExtents().Y() + half_limb_offset), 0.0f),
		vx::Vec3(0.0f, (lower_leg->HalfExtents().Y() + half_limb_offset), 0.0f),
		vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm

				/// right knee
vx::PointConstraint(
	upper_legR,
	lower_legR,
	vx::PointConstraintSettings(
		vx::Vec3(0.0f, -(upper_leg->HalfExtents().Y() + half_limb_offset), 0.0f),
		vx::Vec3(0.0f, (lower_leg->HalfExtents().Y() + half_limb_offset), 0.0f),
		vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm

		};
		//move body upward world frame
		for (auto& b : bodies)
			b->SetPosition(b->Position() + settings.position);
		uint32 count = mPhysicsWorld->GetConstraints().size();
		mPhysicsWorld->CreateConstraintsT(joints, 9);



		if(ragdoll)
		{
			ragdoll->AddBodyPart(lower_legR->GetID(), "Right Upper Leg");
			ragdoll->AddBodyPart(lower_legL->GetID(), "Left Upper Leg");
			ragdoll->AddBodyPart(upper_legR->GetID(), "Right Upper Leg");
			ragdoll->AddBodyPart(upper_legL->GetID(), "Left Upper Leg");
			ragdoll->AddBodyPart(lower_armR->GetID(), "Right Lower Arm");
			ragdoll->AddBodyPart(lower_armL->GetID(), "Left Lower Arm");
			ragdoll->AddBodyPart(_head->GetID(), "Head");
			ragdoll->AddBodyPart(torso->GetID(), "Torso");
			ragdoll->AddBodyPart(upper_armL->GetID(), "Left Upper Arm");
			ragdoll->AddBodyPart(upper_armR->GetID(), "Right Upper Arm");


			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 0], "Neck");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 1], "Left Shoulder");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 2], "Right Shoulder");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 3], "Left Elbow");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 4], "Right Elbow");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 5], "Left Hip");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 6], "Right Hip");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 7], "Left Knee");
			ragdoll->AddConstraint(mPhysicsWorld->GetConstraints()[count + 8], "Right Knee");
		}
	}
	void RagdollBuilder::BuildBoxLimbsSplitTorso(Ragdoll& ragdoll, const RagdollSettings& settings)
	{
		if (!HasLimbShape(ERagdollLimbsShape::Complete_Box_Set))
			GenerateBoxSingleTorsoShapeParts();
		if (!HasLimbShape(ERagdollLimbsShape::Box_Plevis))
			GenerateBoxChestPelvis();


		RefConst<Shape> head = GetLimbShapePart(ERagdollLimbsShape::Box_Head);

		RefConst<Shape> chest = GetLimbShapePart(ERagdollLimbsShape::Box_Chest);
		RefConst<Shape> pelvis = GetLimbShapePart(ERagdollLimbsShape::Box_Plevis);

		RefConst<Shape> upper_arm = GetLimbShapePart(ERagdollLimbsShape::Box_Left_UpperArm);
		RefConst<Shape> lower_arm = GetLimbShapePart(ERagdollLimbsShape::Box_Left_LowerArm);
		RefConst<Shape> upper_leg = GetLimbShapePart(ERagdollLimbsShape::Box_Left_UpperLeg);
		RefConst<Shape> lower_leg = GetLimbShapePart(ERagdollLimbsShape::Box_Left_LowerLeg);

		float chest_half_height = chest->HalfExtents().Y();
		float pelvis_half_height = pelvis->HalfExtents().Y();

		float limb_offset = settings.limbsOffset;

		float chest_plevis_offset = limb_offset * 0.75f;

		Vec3 chestPos(0, pelvis_half_height + chest_half_height + chest_plevis_offset, 0);


		float upper_arm_thickness = upper_arm->HalfExtents().X();
		Vec3 head_pos = chestPos + Vec3(0.0f, head->HalfExtents().Y() + chest_half_height + limb_offset, 0.0f);
		float shoulder_pos_x = chest->HalfExtents().X();// +upper_arm_thickness;
		///pelvis is the origin 
		float shoulder_chest_local_y = (chest->HalfExtents().Y() * 0.6f);
		float shoulder_world_y = chestPos.Y() + shoulder_chest_local_y;


		//float upper_arm_length = upper_arm->HalfExtents().Y(); 
		float upper_arm_half_length = upper_arm->HalfExtents().Y();
		//float lower_arm_length = lower_arm->HalfExtents().Y() * 2.0f; 
		Vec3  upper_armLPos(-(shoulder_pos_x + upper_arm_half_length + limb_offset), shoulder_world_y, 0.0f);
		Vec3  upper_armRPos(upper_armLPos * vx::Vec3(-1, 1, 1));

		Vec3 lower_armLPos = upper_armLPos - Vec3(upper_arm_half_length + lower_arm->HalfExtents().Y() + limb_offset, 0, 0);
		Vec3 lower_armRPos(lower_armLPos * vx::Vec3(-1, 1, 1));

		float pelvis_half_width = pelvis->HalfExtents().X();
		float hipX = pelvis_half_width * 0.45f;
		float upper_leg_half_length = upper_leg->HalfExtents().Y();
		Vec3 upper_legRPos(hipX, -(pelvis_half_height + upper_leg_half_length + limb_offset), 0);
		Vec3 upper_legLPos(upper_legRPos * vx::Vec3(-1, 1, 1));

		float lower_leg_length = lower_leg->HalfExtents().Y();
		Vec3 lower_legRPos = upper_legRPos - Vec3(0, upper_leg_half_length + lower_leg_length + limb_offset, 0);
		Vec3 lower_legLPos(lower_legRPos * vx::Vec3(-1, 1, 1));

		float chest_half_width = chest->HalfExtents().X();

		std::vector<Body*> bodies;
		vx::BodySettings body_settings = vx::BodySettings::DefaultDynamicConstruct();


		///
		float total_masses = 8000.0f;
		float each_masses = total_masses / 10.0f;
		body_settings.overrideMasses = true;
		body_settings.mass = each_masses;
		Float3 v = BodySettings::UnitBoxinteriatensor();
		body_settings.inertia = vx::Float3(v.x * each_masses, v.y * each_masses, v.z * each_masses);

		body_settings.shape = head;
		body_settings.debug_name = "Head";
		body_settings.position = head_pos;
		body_settings.orientation = vx::Quat::Identity();
		body_settings.inertia = body_settings.shape->ComputeInertiaTensorDiagonal(each_masses);
		Body* _head = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(_head);
		ragdoll.AddBodyPart(_head->GetID(), "Head");

		/// body
		body_settings.shape = chest;
		body_settings.debug_name = "Chest";
		body_settings.position = chestPos;
		body_settings.orientation = vx::Quat::Identity();
		body_settings.inertia = body_settings.shape->ComputeInertiaTensorDiagonal(each_masses);
		Body* chest_body = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(chest_body);
		ragdoll.AddBodyPart(chest_body->GetID(), "Chest");

		body_settings.shape = pelvis;
		body_settings.debug_name = "Pelvis";
		body_settings.position = vx::Vec3(0.0f, 0.0f, 0.0f);
		body_settings.orientation = vx::Quat::Identity();
		body_settings.inertia = body_settings.shape->ComputeInertiaTensorDiagonal(each_masses);
		Body* pelvis_body = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(pelvis_body);
		ragdoll.AddBodyPart(pelvis_body->GetID(), "Pelvis");

		/// upper left arm 
		body_settings.shape = upper_arm;
		body_settings.debug_name = "Left Upper Arm";
		body_settings.position = upper_armLPos;
		body_settings.inertia = body_settings.shape->ComputeInertiaTensorDiagonal(each_masses);
		body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
		Body* upper_armL = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(upper_armL);
		ragdoll.AddBodyPart(upper_armL->GetID(), "Left Upper Arm");

		/// upper right arm 
		body_settings.shape = upper_arm;
		body_settings.debug_name = "Right Upper Arm";
		body_settings.position = upper_armRPos;
		body_settings.inertia = body_settings.shape->ComputeInertiaTensorDiagonal(each_masses);
		body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
		Body* upper_armR = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(upper_armR);
		ragdoll.AddBodyPart(upper_armR->GetID(), "Right Upper Arm");

		/// lower left arm 
		body_settings.shape = lower_arm;
		body_settings.debug_name = "Left Lower Arm";
		body_settings.position = lower_armLPos;
		body_settings.inertia = body_settings.shape->ComputeInertiaTensorDiagonal(each_masses);
		body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
		Body* lower_armL = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(lower_armL);
		ragdoll.AddBodyPart(lower_armL->GetID(), "Left Lower Arm");

		/// lower right arm 
		body_settings.shape = lower_arm;
		body_settings.debug_name = "Right Lower Arm";
		body_settings.position = lower_armRPos;
		body_settings.inertia = body_settings.shape->ComputeInertiaTensorDiagonal(each_masses);
		body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
		Body* lower_armR = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(lower_armR);
		ragdoll.AddBodyPart(lower_armR->GetID(), "Right Lower Arm");


		/// upper left leg 
		body_settings.shape = upper_leg;
		body_settings.debug_name = "Left Upper Leg";
		body_settings.position = upper_legLPos;
		body_settings.inertia = body_settings.shape->ComputeInertiaTensorDiagonal(each_masses);
		body_settings.orientation = vx::Quat::Identity();
		Body* upper_legL = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(upper_legL);
		ragdoll.AddBodyPart(upper_legL->GetID(), "Left Upper Leg");

		/// upper right leg 
		body_settings.shape = upper_leg;
		body_settings.debug_name = "Right Upper Leg";
		body_settings.position = upper_legRPos;
		body_settings.inertia = body_settings.shape->ComputeInertiaTensorDiagonal(each_masses);
		body_settings.orientation = vx::Quat::Identity();
		Body* upper_legR = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(upper_legR);
		ragdoll.AddBodyPart(upper_legR->GetID(), "Right Upper Leg");


		/// lowe left leg 
		body_settings.shape = lower_leg;
		body_settings.debug_name = "Left Lower Leg";
		body_settings.position = lower_legLPos;
		body_settings.inertia = body_settings.shape->ComputeInertiaTensorDiagonal(each_masses);
		body_settings.orientation = vx::Quat::Identity();
		Body* lower_legL = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(lower_legL);
		ragdoll.AddBodyPart(lower_legL->GetID(), "Left Lower Leg");

		/// lower right leg 
		body_settings.shape = lower_leg;
		body_settings.debug_name = "Right Lower Leg";
		body_settings.position = lower_legRPos;
		body_settings.inertia = body_settings.shape->ComputeInertiaTensorDiagonal(each_masses);
		body_settings.orientation = vx::Quat::Identity();
		Body* lower_legR = mPhysicsWorld->CreateBody(body_settings);
		bodies.push_back(lower_legR);
		ragdoll.AddBodyPart(lower_legL->GetID(), "Right Lower Leg");


		float half_limb_offset = limb_offset * 0.5f;
		float half_chest_plevis_offset = chest_plevis_offset * 0.5f;

		vx::PointConstraint joints[] =
		{
			vx::PointConstraint(_head,
				chest_body,
				vx::PointConstraintSettings(vx::Vec3(0.0f, -(head->HalfExtents().Y() + half_limb_offset), 0.0f),
					vx::Vec3(0.0f, chest_half_height + half_limb_offset, 0.0f), vx::EConstraintFrame::Local)), ///neck_joint

			vx::PointConstraint(
				chest_body,
				pelvis_body,
				vx::PointConstraintSettings(
					vx::Vec3(0.0f, -(chest_half_height + (half_chest_plevis_offset)), 0.0f),
					vx::Vec3(0.0f, pelvis_half_height + (half_chest_plevis_offset), 0.0f),
					vx::EConstraintFrame::Local)), ///left_shoulder_body_upper_left_arm

				///upper left arm
		vx::PointConstraint(
			upper_armL,
			chest_body,
			vx::PointConstraintSettings(
				vx::Vec3(0.0f, -(upper_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
				vx::Vec3(-(chest_half_width + half_limb_offset), shoulder_chest_local_y, 0.0f),
				vx::EConstraintFrame::Local)), ///left_shoulder_body_upper_left_arm

				///upper right arm
		vx::PointConstraint(
			upper_armR,
			chest_body,
			vx::PointConstraintSettings(
				vx::Vec3(0.0f, -(upper_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
				vx::Vec3(chest_half_width + half_limb_offset, shoulder_chest_local_y, 0.0f),
				vx::EConstraintFrame::Local)), ///left_arm__upper_left_lower_left_arn

				///lower left arm
		vx::PointConstraint(
			upper_armL,
			lower_armL,
			vx::PointConstraintSettings(
				vx::Vec3(0.0f, (upper_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
				vx::Vec3(0.0f, -(lower_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
				vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm

				///lower right arm elbow
				vx::PointConstraint(
					upper_armR,
					lower_armR,
					vx::PointConstraintSettings(
				vx::Vec3(0.0f, (upper_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
					vx::Vec3(0.0f, -(lower_arm->HalfExtents().Y() + half_limb_offset), 0.0f),
					vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm


				/// left hip
vx::PointConstraint(
	pelvis_body,
	upper_legL,
	vx::PointConstraintSettings(
		vx::Vec3(-hipX, -(pelvis_half_height + half_limb_offset), 0.0f),
		vx::Vec3(0.0f, upper_leg->HalfExtents().Y() + half_limb_offset, 0.0f),
		vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm


				/// right hip
vx::PointConstraint(
	pelvis_body,
	upper_legR,
	vx::PointConstraintSettings(
		vx::Vec3(hipX, -(pelvis_half_height + half_limb_offset), 0.0f),
		vx::Vec3(0.0f, upper_leg->HalfExtents().Y() + half_limb_offset, 0.0f),
		vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm


				/// left knee
vx::PointConstraint(
	upper_legL,
	lower_legL,
	vx::PointConstraintSettings(
		vx::Vec3(0.0f, -(upper_leg->HalfExtents().Y() + half_limb_offset), 0.0f),
		vx::Vec3(0.0f, (lower_leg->HalfExtents().Y() + half_limb_offset), 0.0f),
		vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm

				/// right knee
vx::PointConstraint(
	upper_legR,
	lower_legR,
	vx::PointConstraintSettings(
		vx::Vec3(0.0f, -(upper_leg->HalfExtents().Y() + half_limb_offset), 0.0f),
		vx::Vec3(0.0f, (lower_leg->HalfExtents().Y() + half_limb_offset), 0.0f),
		vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm

		};

		uint32 count = mPhysicsWorld->GetConstraints().size();
		mPhysicsWorld->CreateConstraintsT(joints, 10);

		ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count + 0], "Neck");
		ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count + 1], "Spine");
		ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count + 2], "Left Shoulder");
		ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count + 3], "Right Shoulder");
		ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count + 4], "Left Elbow");
		ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count + 5], "Right Elbow");
		ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count + 6], "Left Hip");
		ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count + 7], "Right Hip");
		ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count + 8], "Left Knee");
		ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count + 9], "Right Knee");

		//move body upward world frame
		for (auto& b : bodies)
			b->SetPosition(b->Position() + settings.position);
	}
} //namespace vx
