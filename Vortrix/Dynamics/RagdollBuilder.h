#pragma once
#include "Core/Core.h"
#include "Collision/Shapes/Shape.h"
#include "Body/BodyID.h"

/// <summary>
/// This is a Ragdoll builder class, that builds 
/// ragdolll limbs part, which are not managed for now 
/// 
/// a class could be stored by user, to track limbs part but its not 
/// used to add, move, modify the ragdoll, mainly debuging
/// </summary>

namespace vx
{

	class Constraint;
	class PhysicsWorld;

	struct RagdollSettings
	{
		Vec3 position = Vec3(0.0f);
		float limbsOffset = 0.15f;

		bool splitTorso = true;
		EShapeType mShapesType = EShapeType::Box;
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

		void AddBodyPart(const BodyID id, const char* name);
		void AddConstraint(Constraint* constraint, const char* name);

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

			Capsule_Head,
			Capsule_Torso,

			Capsule_Left_UpperArm,
			Capsule_Left_LowerArm,

			Capsule_Right_UpperArm = Capsule_Left_UpperArm,
			Capsule_Right_LowerArm = Capsule_Left_LowerArm,

			Capsule_Left_UpperLeg,
			Capsule_Left_LowerLeg,

			Capsule_Right_UpperLeg = Capsule_Left_UpperLeg,
			Capsule_Right_LowerLeg = Capsule_Left_LowerLeg,

			Complete_Capsule_Set = Capsule_Right_LowerLeg,

			TOTAL,
		};

		void GenerateBoxSingleTorsoShapeParts();

		void GenerateBoxChestPelvis();

		void GenerateCapsuleSingleTorsoShapeParts();


		RefConst<Shape> GetLimbShapePart(ERagdollLimbsShape part)
		{
			VX_ASSERT(int(part) < mCacheLimbShapes.size());
			return mCacheLimbShapes[int(part)];
		}

		bool HasLimbShape(ERagdollLimbsShape limb) const { return mCacheLimbShapes[size_t(limb)] != nullptr; }

		void BuildBoxLimbsSingleTorso(const RagdollSettings& settings, Ragdoll* ragdoll);


		void BuildCapsuleLimbsSingleTorso(const RagdollSettings& settings, Ragdoll* ragdoll);

		Ragdoll Build(const RagdollSettings& settings)
		{
			VX_ASSERT(settings.mShapesType == EShapeType::Box || settings.mShapesType == EShapeType::Capsule, "Ragdoll shape type has to be either a box or capsule");
			Ragdoll rag_doll;
			if (settings.splitTorso)
				BuildBoxLimbsSplitTorso(rag_doll, settings);
			else
			{
				if (settings.mShapesType == EShapeType::Box)
					BuildBoxLimbsSingleTorso(settings, &rag_doll);
				else if (settings.mShapesType == EShapeType::Capsule)
					BuildCapsuleLimbsSingleTorso(settings, &rag_doll);
			}

			return rag_doll;
		}


		void BuildBoxLimbsSplitTorso(Ragdoll& ragdoll, const RagdollSettings& settings);

	private:
		PhysicsWorld* mPhysicsWorld;
		std::array<vx::RefConst<Shape>, size_t(ERagdollLimbsShape::TOTAL)> mCacheLimbShapes;
	};
} //namespace vx
