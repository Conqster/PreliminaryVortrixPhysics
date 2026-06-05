#pragma once

#include <vector>

struct GLFWwindow;

namespace vx{
	namespace Particles {
		class Particle;
	}
}
class Texture;

#include "Vortrix.h"


#include "Vortrix/Dynamics/Body/EBodyDebugFlags.h"
#include "Vortrix/Dynamics/Body/EDynamicsDofs.h"
namespace vx{
	class PhysicsWorld;
	class BodyManager;
	class Body;
	class BodyDebug;
	enum class EMotionType : uint8;

	class Constraint;
	class DistanceConstraint;
	class PointConstraint;
}

class EditorImGui
{
public:
	void Initialise(GLFWwindow* glfw_win);
	void BeginNewFrame();
	void RenderFrame();
	void Shutdown();

	bool UIBlockingMouseInput() const;
	bool UIBlockingInput();

	void DrawParticlesOverlayItems(std::vector<vx::Particles::Particle>& particles) const;
	void DrawBodiesOverlayItems(vx::BodyManager& body_manager, vx::PhysicsWorld* physics_world);
	void DrawConstraintsOverlayItems(vx::BodyManager& body_manager, std::vector<vx::Constraint*>& constraints);


	static bool EditQuatWithDrag(vx::Quat& quat, bool& editing,
		vx::Vec2& pad_size);

	static bool EditEulerWithDrag(vx::Vec3& euler, bool& editing, vx::Vec2& pad_size, const void* tex);


	static bool ColourEdit3(const char* label, vx::Colour& c);
	static bool ColourEdit4(const char* label, vx::Colour& c);
	

	template<typename Enum>
	static bool Combo(const char* label, Enum& value, const char* items_separated_by_zeros, int height_in_items = -1)
	{
		int _v = static_cast<int>(value);
		bool changed = InternalCombo(label, &_v, items_separated_by_zeros, height_in_items);
		if (changed)
			value = static_cast<Enum>(_v);
		return changed;
	}

private: 
	static bool InternalCombo(const char* label, int* current_item, const char* items_separated_by_zeros, int height_in_items = -1);
	

	template<vx::EMotionType Type>
	void DrawBodyOverlayDetails(vx::Body& body, vx::BodyDebug& body_debug_info);
	static void DrawBodyFlags(vx::EBodySimphaseFlags flags);
	static bool DrawAllowedDofFlags(vx::EDynamicsDofs& flags);


	void DrawDistanceConstraintOverlayUniqueProps(vx::DistanceConstraint& constraint);
	void DrawPointConstraintOverlayUniqueProps(vx::PointConstraint& constraint);
};



