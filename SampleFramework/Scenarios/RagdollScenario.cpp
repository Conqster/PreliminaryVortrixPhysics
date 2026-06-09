#include "RagdollScenario.h"
#include "PhysicsWorld.h"

#include "Collision/Shapes/BoxShape.h"

#include "Dynamics/Constraints/PointConstraint.h"


#include "external/imgui/imgui.h"


namespace vx
{

	struct RagdollSettings
	{
		Vec3 position = Vec3(0.0f);
		float limbsOffset = 0.15f;

		bool splitTorso = true;
	};


	/// for ease debugging
	class Ragdoll
	{
	public:
		using BodyIDVector = std::vector<BodyID>;

		/// for limb part/joint idx to name
		struct IdxToName
		{
			int idx;
			StackString<20> name;
		};

		void AddBodyPart(const BodyID id, const char* name)
		{
			mBodyIDs.push_back(id);
			mBodyNames.push_back({ int(mBodyIDs.size() - 1), StackString<20>(name) });
		}

		void AddConstraint(Constraint* constraint, const char* name)
		{
			mConstraints.push_back(constraint);
			mConstraintNames.push_back({ int(mConstraints.size() - 1), StackString<20>(name) });
		}

	//private:
		BodyIDVector mBodyIDs;
		std::vector<Constraint*> mConstraints;

		std::vector<IdxToName> mBodyNames;
		std::vector<IdxToName> mConstraintNames;
	};


	class RagdollBuilder
	{
	public:
		RagdollBuilder(PhysicsWorld* physics_world) : mPhysicsWorld(physics_world) {}

		enum class ERagdollLimbsShape
		{
			Box_Head = 0,
			Box_Torso, 

			Box_Left_UpperArm,
			Box_Left_LowerArm,

			Box_Right_UpperArm = Box_Left_UpperArm,
			Box_Right_LowerArm = Box_Left_LowerArm,

			Box_Left_UpperLeg,
			Box_Left_LowerLeg,

			Box_Right_UpperLeg = Box_Left_UpperLeg,
			Box_Right_LowerLeg = Box_Left_LowerLeg,

			Complete_Box_Set = Box_Right_LowerLeg,

			Box_Chest,
			Box_Plevis,

			TOTAL,

			Capsule_Head = 7
		};

		void GenerateBoxSingleTorsoShapeParts()
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

		void GenerateBoxChestPelvis()
		{
			VX_ASSERT(!HasLimbShape(ERagdollLimbsShape::Box_Chest));
			//vx::BoxShapeSettings chest_settings(Vec3(0.38f, 0.35f, 0.20f));
			//vx::BoxShapeSettings pelvis_settings(Vec3(0.30f, 0.20f, 0.18f));

			vx::BoxShapeSettings chest_settings(Vec3(0.25f, 0.2f, 0.10f));
			vx::BoxShapeSettings pelvis_settings(Vec3(0.25f, 0.10f, 0.1f));


			float each_density = 12000 * 5.0f;
			chest_settings.SetDensity(each_density);
			pelvis_settings.SetDensity(each_density);

			mCacheLimbShapes[size_t(ERagdollLimbsShape::Box_Chest)] = vx::MakeRef<vx::BoxShape>(chest_settings);
			mCacheLimbShapes[size_t(ERagdollLimbsShape::Box_Plevis)] = vx::MakeRef<vx::BoxShape>(pelvis_settings);
		}


		RefConst<Shape> GetLimbShapePart(ERagdollLimbsShape part) 
		{
			VX_ASSERT(int(part) < mCacheLimbShapes.size());
			return mCacheLimbShapes[int(part)];
		}

		bool HasLimbShape(ERagdollLimbsShape limb) const	{ return mCacheLimbShapes[size_t(limb)] != nullptr; }

		Ragdoll BuildBoxLimbsSingleTorso(const RagdollSettings& settings)
		{
			if (!HasLimbShape(ERagdollLimbsShape::Complete_Box_Set))
				GenerateBoxSingleTorsoShapeParts();


			RefConst<Shape> head = GetLimbShapePart(ERagdollLimbsShape::Box_Head);
			RefConst<Shape> torso_shape = GetLimbShapePart(ERagdollLimbsShape::Box_Torso);
			RefConst<Shape> upper_arm = GetLimbShapePart(ERagdollLimbsShape::Box_Left_UpperArm);
			RefConst<Shape> lower_arm = GetLimbShapePart(ERagdollLimbsShape::Box_Left_LowerArm);
			RefConst<Shape> upper_leg = GetLimbShapePart(ERagdollLimbsShape::Box_Left_UpperLeg);
			RefConst<Shape> lower_leg = GetLimbShapePart(ERagdollLimbsShape::Box_Left_LowerLeg);

			Vec3 torso_half_size = torso_shape->GetHalfExtents();

			float limb_offset = settings.limbsOffset;

			float upper_arm_thickness = upper_arm->GetHalfExtents().X();
			Vec3 head_pos(0.0f, head->GetHalfExtents().Y() + torso_shape->GetHalfExtents().Y() + limb_offset, 0.0f);
			float shoulder_x = torso_shape->GetHalfExtents().X();// +upper_arm_thickness;
			float shoulder_y = torso_shape->GetHalfExtents().Y() * 0.6f;

			//float upper_arm_length = upper_arm->GetHalfExtents().Y(); 
			float upper_arm_half_length = upper_arm->GetHalfExtents().Y(); 
			//float lower_arm_length = lower_arm->GetHalfExtents().Y() * 2.0f; 
			Vec3  upper_armLPos(-(shoulder_x + upper_arm_half_length + limb_offset), shoulder_y, 0.0f);
			Vec3  upper_armRPos(upper_armLPos * vx::Vec3(-1, 1,1));

			Vec3 lower_armLPos = upper_armLPos - Vec3(upper_arm_half_length + lower_arm->GetHalfExtents().Y() + limb_offset, 0, 0);
			Vec3 lower_armRPos(lower_armLPos * vx::Vec3(-1, 1, 1));

			float torso_half_width = torso_half_size.X();
			float torso_half_height = torso_half_size.Y();
			float hipX = torso_half_width * 0.45f;
			float upper_leg_half_length = upper_leg->GetHalfExtents().Y();
			Vec3 upper_legRPos(hipX, -(torso_half_height + upper_leg_half_length + limb_offset), 0);
			Vec3 upper_legLPos(upper_legRPos * vx::Vec3(-1, 1, 1));


			float lower_leg_length = lower_leg->GetHalfExtents().Y();
			Vec3 lower_legRPos = upper_legRPos - Vec3(0, upper_leg_half_length + lower_leg_length + limb_offset, 0);
			Vec3 lower_legLPos(lower_legRPos * vx::Vec3(-1, 1, 1));


			std::vector<Body*> bodies;
			vx::BodySettings body_settings = vx::BodySettings::DefaultDynamicConstruct();
			Ragdoll ragdoll;

			
			body_settings.shape = head;
			body_settings.debug_name = "Head";
			body_settings.position = head_pos;
			body_settings.orientation = vx::Quat::Identity();
			Body* _head = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(_head);
			ragdoll.AddBodyPart(_head->GetID(), "Head");

			/// body
			body_settings.shape = torso_shape;
			body_settings.debug_name = "Torso";
			body_settings.position = vx::Vec3(0.0f, 0.0f, 0.0f);
			body_settings.orientation = vx::Quat::Identity();
			Body* torso = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(torso);
			ragdoll.AddBodyPart(torso->GetID(), "Torso");

			/// upper left arm 
			body_settings.shape = upper_arm;
			body_settings.debug_name = "Left Upper Arm";
			body_settings.position = upper_armLPos;
			body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
			Body* upper_armL = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(upper_armL);
			ragdoll.AddBodyPart(upper_armL->GetID(), "Left Upper Arm");

			/// upper right arm 
			body_settings.shape = upper_arm;
			body_settings.debug_name = "Right Upper Arm";
			body_settings.position = upper_armRPos;
			body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
			Body* upper_armR = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(upper_armR);
			ragdoll.AddBodyPart(upper_armR->GetID(), "Right Upper Arm");

			/// lower left arm 
			body_settings.shape = lower_arm;
			body_settings.debug_name = "Left Lower Arm";
			body_settings.position = lower_armLPos;
			body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
			Body* lower_armL = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(lower_armL);
			ragdoll.AddBodyPart(lower_armL->GetID(), "Left Lower Arm");

			/// lower right arm 
			body_settings.shape = lower_arm;
			body_settings.debug_name = "Right Lower Arm";
			body_settings.position = lower_armRPos;
			body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
			Body* lower_armR = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(lower_armR);
			ragdoll.AddBodyPart(lower_armR->GetID(), "Right Lower Arm");



			/// upper left leg 
			body_settings.shape = upper_leg;
			body_settings.debug_name = "Left Upper Leg";
			body_settings.position = upper_legLPos;
			body_settings.orientation = vx::Quat::Identity();
			Body* upper_legL = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(upper_legL);
			ragdoll.AddBodyPart(upper_legL->GetID(), "Left Upper Leg");

			/// upper right leg 
			body_settings.shape = upper_leg;
			body_settings.debug_name = "Right Upper Leg";
			body_settings.position = upper_legRPos;
			body_settings.orientation = vx::Quat::Identity();
			Body* upper_legR = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(upper_legR);
			ragdoll.AddBodyPart(upper_legR->GetID(), "Right Upper Leg");


			/// lowe left leg 
			body_settings.shape = lower_leg;
			body_settings.debug_name = "Left Lower Leg";
			body_settings.position = lower_legLPos;
			body_settings.orientation = vx::Quat::Identity();
			Body* lower_legL = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(lower_legL);
			ragdoll.AddBodyPart(lower_legL->GetID(), "Left Upper Leg");

			/// lower right leg 
			body_settings.shape = lower_leg;
			body_settings.debug_name = "Right Lower Leg";
			body_settings.position = lower_legRPos;
			body_settings.orientation = vx::Quat::Identity();
			Body* lower_legR = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(lower_legR);
			ragdoll.AddBodyPart(lower_legR->GetID(), "Right Upper Leg");


			float half_limb_offset = limb_offset *0.5f;

			vx::PointConstraint joints[] =
			{

				vx::PointConstraint(_head, 
					torso,
					vx::PointConstraintSettings(vx::Vec3(0.0f, -(head->GetHalfExtents().Y() + half_limb_offset), 0.0f),
						vx::Vec3(0.0f, torso_half_height + half_limb_offset, 0.0f), vx::EConstraintFrame::Local)), ///neck_joint

						///upper left arm
				vx::PointConstraint(
					upper_armL,
					torso,
					vx::PointConstraintSettings(
						vx::Vec3(0.0f, -(upper_arm->GetHalfExtents().Y() + half_limb_offset), 0.0f),
						vx::Vec3(-(torso_half_width + half_limb_offset), shoulder_y, 0.0f),
						vx::EConstraintFrame::Local)), ///left_shoulder_body_upper_left_arm

					///upper right arm
			vx::PointConstraint(
				upper_armR,
				torso,
					vx::PointConstraintSettings(
						vx::Vec3(0.0f, -(upper_arm->GetHalfExtents().Y() + half_limb_offset), 0.0f),
						vx::Vec3(torso_half_width + half_limb_offset, shoulder_y, 0.0f),
						vx::EConstraintFrame::Local)), ///left_arm__upper_left_lower_left_arn

						///lower left arm
				vx::PointConstraint(
					upper_armL,
					lower_armL,
					vx::PointConstraintSettings(
						vx::Vec3(0.0f, (upper_arm->GetHalfExtents().Y()+ half_limb_offset), 0.0f),
						vx::Vec3(0.0f, -(lower_arm->GetHalfExtents().Y() + half_limb_offset), 0.0f),
						vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm

					///lower right arm elbow
					vx::PointConstraint(
						upper_armR,
						lower_armR,
						vx::PointConstraintSettings(
					vx::Vec3(0.0f, (upper_arm->GetHalfExtents().Y() + half_limb_offset), 0.0f),
						vx::Vec3(0.0f, -(lower_arm->GetHalfExtents().Y() + half_limb_offset), 0.0f),
						vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm


											/// left hip
				vx::PointConstraint(
					torso,
					upper_legL,
					vx::PointConstraintSettings(
						vx::Vec3(-hipX, -(torso_half_height + half_limb_offset), 0.0f),
						vx::Vec3(0.0f, upper_leg->GetHalfExtents().Y() + half_limb_offset, 0.0f),
						vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm


					/// right hip
vx::PointConstraint(
	torso,
					upper_legR,
					vx::PointConstraintSettings(
						vx::Vec3(hipX, -(torso_half_height + half_limb_offset), 0.0f),
						vx::Vec3(0.0f, upper_leg->GetHalfExtents().Y() + half_limb_offset, 0.0f),
						vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm

					
											/// left knee
				vx::PointConstraint(
					upper_legL,
					lower_legL,
					vx::PointConstraintSettings(
						vx::Vec3(0.0f, -(upper_leg->GetHalfExtents().Y() + half_limb_offset), 0.0f),
						vx::Vec3(0.0f, (lower_leg->GetHalfExtents().Y() + half_limb_offset), 0.0f),
						vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm

												/// right knee
				vx::PointConstraint(
					upper_legR,
					lower_legR,
					vx::PointConstraintSettings(
						vx::Vec3(0.0f, -(upper_leg->GetHalfExtents().Y() + half_limb_offset), 0.0f),
						vx::Vec3(0.0f, (lower_leg->GetHalfExtents().Y() + half_limb_offset), 0.0f),
						vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm

			};
			//move body upward world frame
			for (auto& b : bodies)
				b->SetPosition(b->GetPosition() + settings.position);
			uint32 count = mPhysicsWorld->GetConstraints().size();
			mPhysicsWorld->CreateConstraintsT(joints, 9);


			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+0], "Neck");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+2], "Left Shoulder");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+3], "Right Shoulder");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+4], "Left Elbow");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+5], "Right Elbow");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+6], "Left Hip");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+7], "Right Hip");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+8], "Left Knee");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+9], "Right Knee");

			return ragdoll;
		}


		Ragdoll Build2(const RagdollSettings& settings)
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

			float chest_half_height = chest->GetHalfExtents().Y();
			float pelvis_half_height = pelvis->GetHalfExtents().Y();
			 
			float limb_offset = settings.limbsOffset;

			float chest_plevis_offset = limb_offset * 0.75f;

			Vec3 chestPos(0, pelvis_half_height + chest_half_height + chest_plevis_offset, 0);


			float upper_arm_thickness = upper_arm->GetHalfExtents().X();
			Vec3 head_pos = chestPos + Vec3(0.0f, head->GetHalfExtents().Y() + chest_half_height + limb_offset, 0.0f);
			float shoulder_pos_x = chest->GetHalfExtents().X();// +upper_arm_thickness;
			///pelvis is the origin 
			float shoulder_chest_local_y = (chest->GetHalfExtents().Y() * 0.6f);
			float shoulder_world_y = chestPos.Y() + shoulder_chest_local_y;


			//float upper_arm_length = upper_arm->GetHalfExtents().Y(); 
			float upper_arm_half_length = upper_arm->GetHalfExtents().Y();
			//float lower_arm_length = lower_arm->GetHalfExtents().Y() * 2.0f; 
			Vec3  upper_armLPos(-(shoulder_pos_x + upper_arm_half_length + limb_offset), shoulder_world_y, 0.0f);
			Vec3  upper_armRPos(upper_armLPos * vx::Vec3(-1, 1, 1));

			Vec3 lower_armLPos = upper_armLPos - Vec3(upper_arm_half_length + lower_arm->GetHalfExtents().Y() + limb_offset, 0, 0);
			Vec3 lower_armRPos(lower_armLPos * vx::Vec3(-1, 1, 1));

			float pelvis_half_width = pelvis->GetHalfExtents().X();
			float hipX = pelvis_half_width * 0.45f;
			float upper_leg_half_length = upper_leg->GetHalfExtents().Y();
			Vec3 upper_legRPos(hipX, -(pelvis_half_height + upper_leg_half_length + limb_offset), 0);
			Vec3 upper_legLPos(upper_legRPos * vx::Vec3(-1, 1, 1));

			float lower_leg_length = lower_leg->GetHalfExtents().Y();
			Vec3 lower_legRPos = upper_legRPos - Vec3(0, upper_leg_half_length + lower_leg_length + limb_offset, 0);
			Vec3 lower_legLPos(lower_legRPos * vx::Vec3(-1, 1, 1));

			float chest_half_width = chest->GetHalfExtents().X();

			std::vector<Body*> bodies;
			vx::BodySettings body_settings = vx::BodySettings::DefaultDynamicConstruct();
			Ragdoll ragdoll;

			body_settings.shape = head;
			body_settings.debug_name = "Head";
			body_settings.position = head_pos;
			body_settings.orientation = vx::Quat::Identity();
			Body* _head = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(_head);
			ragdoll.AddBodyPart(_head->GetID(), "Head");

			/// body
			body_settings.shape = chest;
			body_settings.debug_name = "Chest";
			body_settings.position = chestPos;
			body_settings.orientation = vx::Quat::Identity();
			Body* chest_body = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(chest_body);
			ragdoll.AddBodyPart(chest_body->GetID(), "Chest");

			body_settings.shape = pelvis;
			body_settings.debug_name = "Pelvis";
			body_settings.position = vx::Vec3(0.0f, 0.0f, 0.0f);
			body_settings.orientation = vx::Quat::Identity();
			Body* pelvis_body = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(pelvis_body);
			ragdoll.AddBodyPart(pelvis_body->GetID(), "Pelvis");

			/// upper left arm 
			body_settings.shape = upper_arm;
			body_settings.debug_name = "Left Upper Arm";
			body_settings.position = upper_armLPos;
			body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
			Body* upper_armL = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(upper_armL);
			ragdoll.AddBodyPart(upper_armL->GetID(), "Left Upper Arm");

			/// upper right arm 
			body_settings.shape = upper_arm;
			body_settings.debug_name = "Right Upper Arm";
			body_settings.position = upper_armRPos;
			body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
			Body* upper_armR = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(upper_armR);
			ragdoll.AddBodyPart(upper_armR->GetID(), "Right Upper Arm");

			/// lower left arm 
			body_settings.shape = lower_arm;
			body_settings.debug_name = "Left Lower Arm";
			body_settings.position = lower_armLPos;
			body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
			Body* lower_armL = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(lower_armL);
			ragdoll.AddBodyPart(lower_armL->GetID(), "Left Lower Arm");

			/// lower right arm 
			body_settings.shape = lower_arm;
			body_settings.debug_name = "Right Lower Arm";
			body_settings.position = lower_armRPos;
			body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
			Body* lower_armR = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(lower_armR);
			ragdoll.AddBodyPart(lower_armR->GetID(), "Right Lower Arm");


			/// upper left leg 
			body_settings.shape = upper_leg;
			body_settings.debug_name = "Left Upper Leg";
			body_settings.position = upper_legLPos;
			body_settings.orientation = vx::Quat::Identity();
			Body* upper_legL = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(upper_legL);
			ragdoll.AddBodyPart(upper_legL->GetID(), "Left Upper Leg");

			/// upper right leg 
			body_settings.shape = upper_leg;
			body_settings.debug_name = "Right Upper Leg";
			body_settings.position = upper_legRPos;
			body_settings.orientation = vx::Quat::Identity();
			Body* upper_legR = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(upper_legR);
			ragdoll.AddBodyPart(upper_legR->GetID(), "Right Upper Leg");


			/// lowe left leg 
			body_settings.shape = lower_leg;
			body_settings.debug_name = "Left Lower Leg";
			body_settings.position = lower_legLPos;
			body_settings.orientation = vx::Quat::Identity();
			Body* lower_legL = mPhysicsWorld->CreateBody(body_settings);
			bodies.push_back(lower_legL);
			ragdoll.AddBodyPart(lower_legL->GetID(), "Left Lower Leg");

			/// lower right leg 
			body_settings.shape = lower_leg;
			body_settings.debug_name = "Right Lower Leg";
			body_settings.position = lower_legRPos;
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
					vx::PointConstraintSettings(vx::Vec3(0.0f, -(head->GetHalfExtents().Y() + half_limb_offset), 0.0f),
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
						vx::Vec3(0.0f, -(upper_arm->GetHalfExtents().Y() + half_limb_offset), 0.0f),
						vx::Vec3(-(chest_half_width + half_limb_offset), shoulder_chest_local_y, 0.0f),
						vx::EConstraintFrame::Local)), ///left_shoulder_body_upper_left_arm

					///upper right arm
			vx::PointConstraint(
				upper_armR,
				chest_body,
				vx::PointConstraintSettings(
					vx::Vec3(0.0f, -(upper_arm->GetHalfExtents().Y() + half_limb_offset), 0.0f),
					vx::Vec3(chest_half_width + half_limb_offset, shoulder_chest_local_y, 0.0f),
					vx::EConstraintFrame::Local)), ///left_arm__upper_left_lower_left_arn

					///lower left arm
			vx::PointConstraint(
				upper_armL,
				lower_armL,
				vx::PointConstraintSettings(
					vx::Vec3(0.0f, (upper_arm->GetHalfExtents().Y() + half_limb_offset), 0.0f),
					vx::Vec3(0.0f, -(lower_arm->GetHalfExtents().Y() + half_limb_offset), 0.0f),
					vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm

					///lower right arm elbow
					vx::PointConstraint(
						upper_armR,
						lower_armR,
						vx::PointConstraintSettings(
					vx::Vec3(0.0f, (upper_arm->GetHalfExtents().Y() + half_limb_offset), 0.0f),
						vx::Vec3(0.0f, -(lower_arm->GetHalfExtents().Y() + half_limb_offset), 0.0f),
						vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm


								/// left hip
			vx::PointConstraint(
				pelvis_body,
				upper_legL,
				vx::PointConstraintSettings(
					vx::Vec3(-hipX, -(pelvis_half_height + half_limb_offset), 0.0f),
					vx::Vec3(0.0f, upper_leg->GetHalfExtents().Y() + half_limb_offset, 0.0f),
					vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm
			
			
								/// right hip
			vx::PointConstraint(
				pelvis_body,
				upper_legR,
				vx::PointConstraintSettings(
					vx::Vec3(hipX, -(pelvis_half_height + half_limb_offset), 0.0f),
					vx::Vec3(0.0f, upper_leg->GetHalfExtents().Y() + half_limb_offset, 0.0f),
					vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm
			
			
								/// left knee
			vx::PointConstraint(
				upper_legL,
				lower_legL,
				vx::PointConstraintSettings(
					vx::Vec3(0.0f, -(upper_leg->GetHalfExtents().Y() + half_limb_offset), 0.0f),
					vx::Vec3(0.0f, (lower_leg->GetHalfExtents().Y() + half_limb_offset), 0.0f),
					vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm
			
								/// right knee
			vx::PointConstraint(
				upper_legR,
				lower_legR,
				vx::PointConstraintSettings(
					vx::Vec3(0.0f, -(upper_leg->GetHalfExtents().Y() + half_limb_offset), 0.0f),
					vx::Vec3(0.0f, (lower_leg->GetHalfExtents().Y() + half_limb_offset), 0.0f),
					vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm
			
						};
			
			uint32 count = mPhysicsWorld->GetConstraints().size();
			mPhysicsWorld->CreateConstraintsT(joints, 10);

			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+0], "Neck");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+1], "Spine");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+2], "Left Shoulder");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+3], "Right Shoulder");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+4], "Left Elbow");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+5], "Right Elbow");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+6], "Left Hip");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+7], "Right Hip");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+8], "Left Knee");
			ragdoll.AddConstraint(mPhysicsWorld->GetConstraints()[count+9], "Right Knee");

			//move body upward world frame
			for (auto& b : bodies)
				b->SetPosition(b->GetPosition() + settings.position);

			return ragdoll;
		}


	private:
		PhysicsWorld* mPhysicsWorld;
		std::array<vx::RefConst<Shape>, size_t(ERagdollLimbsShape::TOTAL)> mCacheLimbShapes;
	};
} //namespace vx





std::vector<Ragdoll> mRagdolls;

void RagdollScenario::Init(vx::PhysicsWorld* i_world)
{
	Scenario::Init(i_world);

	//enable to set/override pose
	//RAGDOLL BUILDER

	//make hip the origin

	vx::RagdollBuilder ragdoll_builder(mPhysicsWorld);
	//ragdoll_builder.Build({ vx::Vec3(-5.0f, 4.0f, 0.0f), 0.1f});
	//ragdoll_builder.Build2({vx::Vec3(5.0f, 4.0f, 0.0f), 0.1f});
	//ragdoll_builder.Build({vx::Vec3(0.0f, 4.0f, 0.0f), 0.1f });

	//ragdoll_builder.Build({ vx::Vec3(-5.0f, 4.0f, -4.0f) });
	//ragdoll_builder.Build2({vx::Vec3(5.0f, 4.0f, -4.0f)});
	//ragdoll_builder.Build({vx::Vec3(0.0f, 4.0f, -4.0f)});

	mRagdolls.clear();

	mRagdolls.push_back(ragdoll_builder.Build2({ vx::Vec3(-5.0f, 2.0f, 0.0f), 0.1f }));
	mRagdolls.push_back(ragdoll_builder.Build2({ vx::Vec3(5.0f, 2.0f, 0.0f), 0.1f }));
	mRagdolls.push_back(ragdoll_builder.Build2({ vx::Vec3(0.0f, 2.0f, 0.0f), 0.1f }));

	mRagdolls.push_back(ragdoll_builder.Build2({ vx::Vec3(-5.0f, 2.0f, -4.0f) }));
	mRagdolls.push_back(ragdoll_builder.Build2({ vx::Vec3(5.0f, 2.0f, -4.0f) }));
	mRagdolls.push_back(ragdoll_builder.Build2({ vx::Vec3(0.0f, 2.0f, -4.0f) }));

	//vx::BodySettings body_settings = vx::BodySettings::DefaultDynamicConstruct()765239

	//std::vector<Body*> bodies;

	/////BUILD IN RAGDOLL FRAME
	///// head
	//vx::BoxShapeSettings box_settings(0.5f);
	//body_settings.shape = vx::MakeRef<vx::BoxShape>(box_settings);
	//body_settings.position = vx::Vec3(0.0f, 2.0f, 0.0f);
	//body_settings.orientation = vx::Quat::Identity();
	//Body* head = mPhysicsWorld->CreateBody(body_settings);
	//bodies.push_back(head);
	
	///// body
	//box_settings = vx::BoxShapeSettings(vx::Vec3(1.0f, 1.0f, 0.25f));
	//body_settings.shape = vx::MakeRef<vx::BoxShape>(box_settings);
	//body_settings.position = vx::Vec3(0.0f, 0.0f, 0.0f);
	//body_settings.orientation = vx::Quat::Identity();
	//Body* rag_body = mPhysicsWorld->CreateBody(body_settings);
	//bodies.push_back(rag_body);

	//vx::BoxShapeSettings upper_arm_shape_setting(vx::Vec3(0.25f, 0.75f, 0.25f));
	///// upper left arm 
	//body_settings.shape = vx::MakeRef<vx::BoxShape>(upper_arm_shape_setting);
	//body_settings.position = vx::Vec3(-1.95f, 0.75f, 0.0f);
	//body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
	//Body* upper_armL = mPhysicsWorld->CreateBody(body_settings);
	//bodies.push_back(upper_armL);

	///// lower arm, same length, smaller radius
	//vx::BoxShapeSettings lower_arm_shape_setting(vx::Vec3(0.2f, 0.75f, 0.2f));

	///// lower left arm 
	//body_settings.shape = vx::MakeRef<vx::BoxShape>(lower_arm_shape_setting);
	//body_settings.position = vx::Vec3(-3.65f, 0.75f, 0.0f);
	//body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(90.0f));
	//Body* lower_armL = mPhysicsWorld->CreateBody(body_settings);
	//bodies.push_back(lower_armL);


	///// upper right arm 
	//body_settings.shape = vx::MakeRef<vx::BoxShape>(upper_arm_shape_setting);
	//body_settings.position = vx::Vec3(1.95f, 0.75f, 0.0f);
	//body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
	//Body* upper_armR = mPhysicsWorld->CreateBody(body_settings);
	//bodies.push_back(upper_armR);

	///// lower right arm 
	//body_settings.shape = vx::MakeRef<vx::BoxShape>(lower_arm_shape_setting);
	//body_settings.position = vx::Vec3(3.65f, 0.75f, 0.0f);
	//body_settings.orientation.SetAxisAngle(vx::Vec3::Forward(), vx::DegToRad(-90.0f));
	//Body* lower_armR = mPhysicsWorld->CreateBody(body_settings);
	//bodies.push_back(lower_armR);

	//
	////vx::BoxShapeSettings upper_leg_shape_setting(vx::Vec3(0.35f, 1.0f, 0.35f), 0.0f);
	//vx::BoxShapeSettings upper_leg_shape_setting(vx::Vec3(0.35f, 1.0f, 0.35f));
	///// upper left leg 
	//body_settings.shape = vx::MakeRef<vx::BoxShape>(upper_leg_shape_setting);
	//body_settings.position = vx::Vec3(-0.65f, -2.2f, 0.0f);
	//body_settings.orientation = vx::Quat::Identity();
	//Body* upper_legL = mPhysicsWorld->CreateBody(body_settings);
	//bodies.push_back(upper_legL);

	///// upper right leg 
	//body_settings.shape = vx::MakeRef<vx::BoxShape>(upper_leg_shape_setting);
	//body_settings.position = vx::Vec3(0.65f, -2.2f, 0.0f);
	//body_settings.orientation = vx::Quat::Identity();
	//Body* upper_legR = mPhysicsWorld->CreateBody(body_settings);
	//bodies.push_back(upper_legR);


	//vx::BoxShapeSettings lower_leg_shape_setting(vx::Vec3(0.25f, 1.0f, 0.25f));

	///// lowe left leg 
	//body_settings.shape = vx::MakeRef<vx::BoxShape>(lower_leg_shape_setting);
	//body_settings.position = vx::Vec3(-0.65f, -4.4f, 0.0f);
	//body_settings.orientation = vx::Quat::Identity();
	//Body* lower_legL = mPhysicsWorld->CreateBody(body_settings);
	//bodies.push_back(lower_legL);

	///// lower right leg 
	//body_settings.shape = vx::MakeRef<vx::BoxShape>(lower_leg_shape_setting);
	//body_settings.position = vx::Vec3(0.65f, -4.4f, 0.0f);
	//body_settings.orientation = vx::Quat::Identity();
	//Body* lower_legR = mPhysicsWorld->CreateBody(body_settings);
	//bodies.push_back(lower_legR);


	//float arm_length = 0.85f;
	/////create joints 
	//vx::PointConstraintSettings joint_setting(vx::Vec3(0.0f, -1.1f, 0.0f), vx::Vec3(0.0f, 1.1f, 0.0f), vx::EConstraintFrame::Local);
	//vx::PointConstraint joints[] =
	//{
	//	vx::PointConstraint(head, rag_body, vx::PointConstraintSettings(vx::Vec3(0.0f, -0.75f, 0.0f), vx::Vec3(0.0f, 1.25f, 0.0f), vx::EConstraintFrame::Local)), ///neck_joint
	//	vx::PointConstraint(upper_armL, rag_body, vx::PointConstraintSettings(vx::Vec3(0.0f, -arm_length, 0.0f), vx::Vec3(-1.1f, 0.75f, 0.0f), vx::EConstraintFrame::Local)), ///left_shoulder_body_upper_left_arm
	//	vx::PointConstraint(lower_armL, upper_armL, vx::PointConstraintSettings(vx::Vec3(0.0f, -arm_length, 0.0f), vx::Vec3(0.0f, arm_length, 0.0f), vx::EConstraintFrame::Local)), ///left_arm__upper_left_lower_left_arn
	//	vx::PointConstraint(rag_body, upper_armR, vx::PointConstraintSettings(vx::Vec3(1.1f, 0.75f, 0.0f), vx::Vec3(0.0f, -arm_length, 0.0f), vx::EConstraintFrame::Local)), ///right_shoulder_body_upper_right_arm
	//	vx::PointConstraint(lower_armR, upper_armR, vx::PointConstraintSettings(vx::Vec3(0.0f, -arm_length, 0.0f), vx::Vec3(-0.0f, arm_length, 0.0f), vx::EConstraintFrame::Local)), ///right_arm__upper_right_lower_right_arn
	//	vx::PointConstraint(rag_body, upper_legL,  vx::PointConstraintSettings(vx::Vec3(-0.65f, -1.1f, 0.0f), vx::Vec3(0.0f, 1.1f, 0.0f), vx::EConstraintFrame::Local)), ///neck_joint
	//	vx::PointConstraint(upper_legL, lower_legL, joint_setting), ///neck_joint
	//	vx::PointConstraint(rag_body, upper_legR, vx::PointConstraintSettings(vx::Vec3(0.65f, -1.1f, 0.0f), vx::Vec3(0.0f, 1.1f, 0.0f), vx::EConstraintFrame::Local)), ///neck_joint
	//	vx::PointConstraint(upper_legR, lower_legR, joint_setting), ///neck_joint
	//};

	//mPhysicsWorld->CreateConstraintsT(joints, 9);

	////move body upward world frame
	//for (auto& b : bodies)
	//	b->SetPosition(b->GetPosition() + vx::Vec3(0.0f, 5.5f, 0.0f));
	//	//b->ApplyLinearDisplacement(vx::Vec3(0.0f, 5.0f, 0.0f));


	CreateGroundPlane(100.0f);
}

void RagdollScenario::PostPhysicsStep(float dt)
{
	Scenario::PostPhysicsStep(dt);
}

void RagdollScenario::OnUI()
{
	Scenario::OnUI();

	if (mPhysicsWorld == nullptr)
		return;

	const auto& body_manager = mPhysicsWorld->GetBodyManager();
	if (ImGui::Begin("Ragdolls"))
	{
		for (auto& ragdoll : mRagdolls)
		{
			ImGui::PushID(&ragdoll);

			ImGui::SeparatorText("Limbs");
			for (auto& body_name : ragdoll.mBodyNames)
			{
				ImGui::Text("%d: %s, Body: [%s]", body_name.idx, body_name.name.Data(), body_manager.GetBodyDebugName(ragdoll.mBodyIDs[body_name.idx]));
			}
			ImGui::SeparatorText("joints");
			for (auto& constraint_name : ragdoll.mConstraintNames)
			{
				ImGui::Text("%d: %s, Constraint idx: %d", constraint_name.idx, constraint_name.name.Data(), ragdoll.mConstraints[constraint_name.idx]->ConstraintIdx());
			}

			ImGui::Separator();
			ImGui::PopID();
		}
	}
	ImGui::End();
}
