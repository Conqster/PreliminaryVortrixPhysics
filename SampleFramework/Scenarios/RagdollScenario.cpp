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

	mRagdolls.push_back(ragdoll_builder.Build({ vx::Vec3(-5.0f, 2.0f, 0.0f), 0.1f }));
	mRagdolls.push_back(ragdoll_builder.Build({ vx::Vec3(5.0f, 2.0f, 0.0f), 0.1f, false, vx::EShapeType::Capsule }));
	mRagdolls.push_back(ragdoll_builder.Build({ vx::Vec3(0.0f, 2.0f, 0.0f), 0.1f }));

	mRagdolls.push_back(ragdoll_builder.Build({ vx::Vec3(-5.0f, 2.0f, -4.0f) }));
	mRagdolls.push_back(ragdoll_builder.Build({ vx::Vec3(5.0f, 2.0f, -4.0f) }));
	mRagdolls.push_back(ragdoll_builder.Build({ vx::Vec3(0.0f, 2.0f, -4.0f) }));

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
