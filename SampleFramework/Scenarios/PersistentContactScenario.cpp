#include "PersistentContactScenario.h"
#include "PhysicsWorld.h"

#include "Vortrix/Maths/ScalarMath.h"

#include "Collision/Shapes/Shape.h"
#include "Collision/Shapes/BoxShape.h"

#include "Camera.h"

#include "external/imgui/imgui.h"

void PersistentContactScenario::Init(vx::PhysicsWorld* i_world)
{
	mPhysicsWorld = i_world;
	VX_ASSERT(i_world, "Physics World is null");

	vx::BodySettings dyn_bodies_settings = vx::BodySettings::DefaultDynamicConstruct();

	vx::BoxShape* unit_box = new vx::BoxShape(0.5f);
	dyn_bodies_settings.position = vx::Vec3(0.0f, 5.5f, 0.0f);
	dyn_bodies_settings.debug_name = "box";
	dyn_bodies_settings.shape = new vx::BoxShape(0.5f);
	mPhysicsWorld->CreateBody(dyn_bodies_settings);


	/// Ground plane
	CreateGroundPlane(100.0f);
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