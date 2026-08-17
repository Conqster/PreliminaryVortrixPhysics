#include "RagdollScenario.h"
#include "Vortrix/PhysicsWorld.h"

#include "external/imgui/imgui.h"

#include "Vortrix/Dynamics/RagdollBuilder.h"




std::vector<vx::Ragdoll> mRagdolls;

void RagdollScenario::Init(vx::PhysicsWorld* i_world)
{
	Scenario::Init(i_world);

	//enable to set/override pose
	//RAGDOLL BUILDER

	//make hip the origin

	vx::RagdollBuilder ragdoll_builder(mPhysicsWorld);

	mRagdolls.clear();
	mRagdolls.resize(6);

	vx::RagdollSettings rag_settings;
	rag_settings.position = vx::Vec3(-5.0f, 2.0f, 0.0f);
	rag_settings.limbsOffset = 0.1f;
	CreateRagdoll(rag_settings, &mRagdolls[0]);

	rag_settings.position = vx::Vec3(0.0f, 2.0f, 0.0f);
	CreateRagdoll(rag_settings, &mRagdolls[1]);

	rag_settings.position = vx::Vec3(5.0f, 2.0f, 0.0f);
	rag_settings.splitTorso = false;
	rag_settings.mShapesType = vx::EShapeType::Capsule;
	CreateRagdoll(rag_settings, &mRagdolls[2]);

	rag_settings = vx::RagdollSettings();
	rag_settings.position = vx::Vec3(-5.0f, 2.0f, -4.0f);
	CreateRagdoll(rag_settings, &mRagdolls[3]);

	rag_settings.position = vx::Vec3(5.0f, 2.0f, -4.0f);
	CreateRagdoll(rag_settings, &mRagdolls[4]);

	rag_settings.position = vx::Vec3(0.0f, 2.0f, -4.0f);
	CreateRagdoll(rag_settings, &mRagdolls[5]);


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
