#pragma once

#include "Vortrix/Maths/Core.h"
#include "Vortrix/Maths/Vec3.h"

#include "Vortrix/Core/Colours.h"


#include "Vortrix/Collision/RayCast.h"
#include "Vortrix/Dynamics/Body/BodyID.h"

#include <array>

#include <functional>
namespace vx {
	class PhysicsWorld;
	struct BodySettings;
}
class Camera;
class DebugGizmosRenderer;
class ApplicationWindow;
class EditorImGui;


enum class EClickEvent : vx::uint8
{
	None,
	Down,
	Held, 
	Up,
};


class Scenario
{
public:
	virtual ~Scenario() = default;
	virtual void Init(vx::PhysicsWorld*) = 0;

	virtual const char* Name() = 0;

	virtual const char* Info() = 0;

	virtual void PrePhysicsStep(float dt) {}
	virtual void PostPhysicsStep(float dt);

	virtual void OnUI() {}


	void SetCamera(Camera* cam) { mAppCamera = cam; }
	void SetDebugGizmos(DebugGizmosRenderer* debug_gizmos) { mDebugGizmos = debug_gizmos; }
	void SetWindow(ApplicationWindow* app_win) { mAppWindow = app_win; }
	void SetAppEditor(EditorImGui* ui) { mAppUI = ui; }

	bool GetAllowBaseScenarioMouseCast() const { return mAllowBaseMouseCast; }
	void SetAllowBaseScenarioMouseCast(bool v) { mAllowBaseMouseCast = v; }
protected:
	vx::PhysicsWorld* mPhysicsWorld = nullptr;
	Camera* mAppCamera = nullptr;
	ApplicationWindow* mAppWindow = nullptr;
	DebugGizmosRenderer* mDebugGizmos = nullptr;

	//hack for now
	EditorImGui* mAppUI = nullptr;

	bool mAllowBaseMouseCast = true;
	bool BlockedMouseCastRay();
	void MouseClickCheck();
	void MouseCastRay();

	EClickEvent mMouseEvent = EClickEvent::None;
	vx::BodyID mBody{};
	vx::Vec3 mPointBodyFrame;

	vx::Vec3 mCamFwd;
	float t_dist;

	VX_INLINE std::array<vx::Colour, 3> GetBasisAxisColourArray()
	{
		return {
				vx::Colour::sRed,
				vx::Colour::sGreen,
				vx::Colour::sBlue
		};
	}

	void CreateGroundPlane(float half_size);
	void CreateBoxStack(const vx::BodySettings& body_settings, const vx::Vec3& counts, const vx::Vec3& half_extent, const vx::Vec3& base_pos);
	void CreateBoxPyramidStack(const vx::BodySettings& body_settings, int base_width, int base_depth, int height, const vx::Vec3& half_extent, const vx::Vec3& base_pos);
	void Create1DBoxPyramidStack(const vx::BodySettings& body_settings, int base_width, int base_depth, int height, const vx::Vec3& half_extent, const vx::Vec3& base_pos, vx::Axis shrink_axis);


	void CreateJenga(vx::BodySettings body_setting, const vx::Vec3& half_extent, int layers, vx::Vec3 base_pos, float gap = 0.05f);

	struct Footprint
	{
		explicit Footprint(const vx::Vec3& v) : 
			x(v.X()), y(v.Y()), z(v.Z()){}

		//apply full size along axis x, y,  z, 
		float x;
		float y;
		float z;

		static Footprint BoxFootprint(const vx::Vec3& half_extent)
		{
			return Footprint(2.0f * half_extent);
		}
	};

	struct InterleavePattern
	{
		/// orientation per layer 
		/// so could have stack with rotation 30, 60, 90 or custom etc 
		std::function<vx::Quat(int layer, int i, int count)> Orientation; //orientation per layer,

		std::function<vx::Vec3(vx::Vec3& base, float height_offset, int layer, int i, int count)> Position; //orientation per layer,
		/// orientation is the shape orientation in layer 
		/// direction is the offset direction in layer 
		/// 
		/// might have 
		/// per shape offest within layer 


		std::function<vx::Axis(int layer)> Axis; //X 0; 2 Z

		//direction per layer offset
		std::function<vx::Vec3(int)> Direction;

		static InterleavePattern JengaPattern();
	};
	void CreateInterleavedStructure(vx::BodySettings body_setting, 
		const Footprint& fp,
		int layers, vx::Vec3 base_pos,
		const InterleavePattern& pattern,
		float gap = 0.05f);

	struct StructureConfig
	{
		vx::Vec3 count; //width, hwight, depth
		vx::Vec3 halfExtent;
		vx::Vec3 basePos;

		std::function<vx::Vec2(int height_layer)> GetSize;
		std::function<vx::Vec3(int height_layer)> GetOffset;
		std::function<bool(int x, int y, int z, int w, int d)> PlaceRule;
	};

	void CreateStructure(const vx::BodySettings& body_settings, const StructureConfig& cfg);


};