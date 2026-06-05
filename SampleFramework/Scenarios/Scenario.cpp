#include "Scenario.h"

#include "PhysicsWorld.h"
#include "Vortrix/Collision/Shapes/PlaneShape.h"
#include "Vortrix/Collision/Shapes/BoxShape.h"
#include "Vortrix/Collision/Shapes/SphereShape.h"

#include "SampleFramework/Camera.h"

#include "SampleFramework/EditorImGui.h"
#include "SampleFramework/Display/ApplicationWindow.h"

#include "SampleFramework/Renderer/DebugGizmosRenderer.h"

#include "SampleFramework/Input/InputSystem.h"
#include "Dynamics/Constraints/DistanceConstraint.h"

using namespace InputSystem;

bool Scenario::BlockedMouseCastRay()
{
	if (mAppUI == nullptr) return false;

	bool block = mAppUI->UIBlockingMouseInput();

	if (mAppWindow != nullptr)
	{
		block |= mAppWindow->GetLockCursor();
	}

	return block;
}

void Scenario::MouseClickCheck()
{
	bool last_frame_active = (mMouseEvent == EClickEvent::Down ||
		mMouseEvent == EClickEvent::Down);

	bool clicked = Input::GetMousePressed(IKeyCode::Mouse0);
	bool released = Input::GetMouse(IKeyCode::Mouse0);



	switch (mMouseEvent)
	{
	case EClickEvent::None:
		mMouseEvent = (clicked) ? EClickEvent::Down : EClickEvent::None;
		break;
	case EClickEvent::Down:
		mMouseEvent = (released) ? EClickEvent::Up : EClickEvent::Held;
		break;
	case EClickEvent::Held:
		mMouseEvent = (released) ? EClickEvent::Up : EClickEvent::Held;
		break;
	case EClickEvent::Up:
		mMouseEvent = (clicked) ? EClickEvent::Down : EClickEvent::None;
	default:
		break;
	}

	//if(mMouseEvent == EClickEvent::None)
	//	VX_LOG_DEBUG("Mouse Input: None");
	//else if(mMouseEvent == EClickEvent::Down)
	//	VX_LOG_DEBUG("Mouse Input: Down");
	//else if(mMouseEvent == EClickEvent::Held)
	//	VX_LOG_DEBUG("Mouse Input: Held");
	//else if(mMouseEvent == EClickEvent::Up)
	//	VX_LOG_DEBUG("Mouse Input: Up");
	////else if(mMouseEvent == EClickEvent::Up)

	//expriment
	float dir = Input::GetKeyDown(IKeyCode::T) ? 1.0f :
		Input::GetKeyDown(IKeyCode::Y) ? -1.0f : 0.0f;

	dir = Input::GetScrollWheel();

	t_dist = vx::VxMax(0.0f, t_dist + dir);
	//VX_LOG_DEBUG("Mouse Input: ", Input::GetScrollWheel());

}

void Scenario::MouseCastRay()
{
	if (BlockedMouseCastRay()) return;
	if (mAppCamera == nullptr || mAppWindow == nullptr) return;


	if (mBody.IsValid() && mMouseEvent == EClickEvent::Held) return;

	vx::Vec3 cam_pos = mAppCamera->GetPosition();
	vx::Vec3 cam_fwd = mAppCamera->GetForward();

	vx::Vec2 mouse_cursor_pos = mAppWindow->MouseCursorPosition();

	//if(mouse_cursor_pos.X() < 0.0f && mouse_cursor_pos.Y() < 0.0f)

	//vx::Float3 cursor_ws(mouse_cursor_pos.X(), mouse_cursor_pos.Y(), cam_pos.Z());
	vx::Vec3 cursor_ws(mouse_cursor_pos.X(), mouse_cursor_pos.Y(), -1.0f);
	cursor_ws = mAppCamera->ScreenToWorld(cursor_ws, mAppWindow->GetWidth(), mAppWindow->GetHeight());
	//VX_LOG_DEBUG("Mouse Cursor Pos: ", cursor_ws);
	//vx::Vec3 ray_pos = 

	//cam_fwd = mAppCamera->ViewMat().TransformInverseDirection(cam_fwd);
	cam_fwd = (cursor_ws - cam_pos).Normalised();

	float ray_length = 1000.0f;
	vx::RayCast ray_cast = vx::RayCast(cursor_ws, cam_fwd * ray_length);


	vx::ClosestRaycastHitProcessor processor;
	mPhysicsWorld->GetWorldQuery().CastRay(ray_cast, processor);

	vx::Colour ray_colour = (processor.HasHit()) ? vx::Colour::sRed : vx::Colour::sGreen;
	vx::Vec3 ray_end_point = ray_cast.End();
	if (processor.HasHit())
	{
		vx::RaycastHit hit = processor.Hit();

		vx::Vec3 point = ray_cast.PointAlongRay(hit.fraction);
		ray_end_point = point;

		mDebugGizmos->DrawAACross(point, GetBasisAxisColourArray().data(), 3, 0.2f);

		vx::Body& body = mPhysicsWorld->GetBodyManager().GetBody(hit.body);
		mDebugGizmos->DrawAABB(body.GetAABBWorld(), vx::Colour::sDeepTeal);

		if (!mBody.IsValid() && mMouseEvent != EClickEvent::None)
		{
			if (body.IsSleeping())
				body.WakeUp(-ray_cast.direction * 5.0f);
			mBody = hit.body;
			//transform point to body local
			mPointBodyFrame = Mat44::TransformInverse(
				Mat44::RotationTranslation(body.GetOrientation(), body.GetPosition()), point);

			if(mHasMouseConstraint)
			{
				mMouseDragConstraintSettings.localAnchorA = body.GetOrientation().InverseRotate(point - body.GetPosition());
				mMouseDragConstraint = new vx::DistanceConstraint(&body, mMouseDragBody, mMouseDragConstraintSettings);
				mPhysicsWorld->AddConstraint(mMouseDragConstraint);
				mMouseDragBody->SetPosition(cursor_ws);
			}
		}


		mCamFwd = cam_fwd;
		t_dist = hit.fraction * ray_length;

		mDebugGizmos->DrawAACross(point, GetBasisAxisColourArray().data(), 3, 0.2f);
		mDebugGizmos->DrawArrowCone(point, point + hit.normal, 0.05, 0.1, 0.05, 3, vx::Colour::sCyan);
	}

	mDebugGizmos->DrawLine(ray_cast.origin, ray_end_point, ray_colour);
	if (!ray_cast.End().IsApprox(ray_end_point))
		mDebugGizmos->DrawLine(ray_end_point, ray_cast.End(), vx::Colour::sGreen);
}

void Scenario::Init(vx::PhysicsWorld* i_world)
{
	mPhysicsWorld = i_world;
	VX_ASSERT(i_world, "Physics World is null"); 

	mMouseDragConstraintSettings.minDist = 1.25f;
	mMouseDragConstraintSettings.maxDist = 2.5f;
	mMouseDragConstraintSettings.frequency = vx::DegToRad(120.0f);
	mMouseDragConstraintSettings.dampingRatio = 0.0f;
	mMouseDragConstraintSettings.localAnchorB = {};

	mMouseDragBody = new vx::Body(Vec3(0.0f));
	mMouseDragBody->SetShape(new vx::SphereShape(0.125f));
	mHasMouseConstraint = true;
}

void Scenario::OnClose()
{
	if (mMouseDragBody)
		delete mMouseDragBody->GetShape();
	if (mPhysicsWorld && mMouseDragConstraint)
	{
		mPhysicsWorld->RemoveConstraint(mMouseDragConstraint);
		delete mMouseDragConstraint;
		mMouseDragConstraint = nullptr;
	}
	mHasMouseConstraint = false;
}

void Scenario::PostPhysicsStep(float dt)
{
	MouseClickCheck();
	if (mAllowBaseMouseCast)MouseCastRay();

	if(mHasMouseConstraint)
	{
		if (mBody.IsValid() && mMouseDragConstraint && mMouseEvent == EClickEvent::Held)
		{

			vx::Vec3 cam_pos = mAppCamera->GetPosition();
			vx::Vec3 cam_fwd = mAppCamera->GetForward();
			vx::Vec2 mouse_cursor_pos = mAppWindow->MouseCursorPosition();
			vx::Vec3 cursor_ws(mouse_cursor_pos.X(), mouse_cursor_pos.Y(), -1.0f);
			cursor_ws = mAppCamera->ScreenToWorld(cursor_ws, mAppWindow->GetWidth(), mAppWindow->GetHeight());

			cam_fwd = (cursor_ws - cam_pos).Normalised();
			vx::Vec3 new_pos = cursor_ws + cam_fwd * t_dist;

			mMouseDragBody->SetPosition(new_pos);


			vx::Body& body = mPhysicsWorld->GetBodyManager().GetBody(mBody);
			//cast ray down 
			vx::RayCast ray_cast = vx::RayCast(body.GetPosition(), vx::Vec3(0.0f, -1.0f, 0.0f) * 100.0f);
			vx::AllRaycastHitProcessor<16> processor;
			mPhysicsWorld->GetWorldQuery().CastRay(ray_cast, processor);
			vx::Vec3 end = ray_cast.End();
			if (processor.HasHit())
			{
				processor.Sort();
				vx::RaycastResult result = processor.Result();
				for (int i = 0; i < result.hitCount; ++i)
				{
					vx::RaycastHit hit = processor.Hits()[i];
					if (hit.body != mBody)
					{
						end = ray_cast.PointAlongRay(hit.fraction);

						vx::Colour col = vx::Colour::sYellow;
						mDebugGizmos->DrawAACross(end, &col, 1, 0.3f);
						break;
					}
				}
			}
			mDebugGizmos->DrawLine(ray_cast.origin, end, vx::Colour::sCyan);
		}
		else if (mBody.IsValid() && mMouseDragConstraint && (mMouseEvent == EClickEvent::Up || mMouseEvent == EClickEvent::None))
		{
			vx::Body& body = mPhysicsWorld->GetBodyManager().GetBody(mBody);
			//might just release key jolt body 
			//body.ApplyImpulse(vx::Vec3(0.0f, 1.0f, 0.0f) * 10.0f * (1.0f/body.GetInverseMass()));
			//body.WakeUp(vx::Vec3(0.0f, 1.0f, 0.0f) * 1000.0f);
			mBody = vx::BodyID();

			mPhysicsWorld->RemoveConstraint(mMouseDragConstraint);
			delete mMouseDragConstraint;
			mMouseDragConstraint = nullptr;
		}
		else
		{
			mBody = vx::BodyID();
			if(mMouseDragConstraint)
			{
				mPhysicsWorld->RemoveConstraint(mMouseDragConstraint);
				delete mMouseDragConstraint;
				mMouseDragConstraint = nullptr;
			}
		}
	}
	else
	{
		if (mBody.IsValid() && mMouseEvent == EClickEvent::Held)
		{

			vx::Body& body = mPhysicsWorld->GetBodyManager().GetBody(mBody);
			vx::Vec3 pt_ws = body.GetPosition() + body.GetOrientation().Rotate(mPointBodyFrame);

			//hack
			vx::Vec3 cam_pos = mAppCamera->GetPosition();
			vx::Vec3 cam_fwd = mAppCamera->GetForward();
			vx::Vec2 mouse_cursor_pos = mAppWindow->MouseCursorPosition();
			vx::Vec3 cursor_ws(mouse_cursor_pos.X(), mouse_cursor_pos.Y(), -1.0f);
			cursor_ws = mAppCamera->ScreenToWorld(cursor_ws, mAppWindow->GetWidth(), mAppWindow->GetHeight());

			cam_fwd = (cursor_ws - cam_pos).Normalised();
			vx::Vec3 new_pos = cursor_ws + cam_fwd * t_dist;

			//translate body 
			vx::Vec3 translate_ws = new_pos - body.GetOrientation().Rotate(mPointBodyFrame);
			body.SetPosition(translate_ws);

			mDebugGizmos->DrawAACross(pt_ws, GetBasisAxisColourArray().data(), 3, 0.2f);

			//might move to on select
			body.ClearVelocities();
			body.ClearAccumulatedForces();

			//cast ray down 
			vx::RayCast ray_cast = vx::RayCast(body.GetPosition(), vx::Vec3(0.0f, -1.0f, 0.0f) * 100.0f);
			vx::AllRaycastHitProcessor<16> processor;
			mPhysicsWorld->GetWorldQuery().CastRay(ray_cast, processor);
			vx::Vec3 end = ray_cast.End();
			if (processor.HasHit())
			{
				processor.Sort();
				vx::RaycastResult result = processor.Result();
				for (int i = 0; i < result.hitCount; ++i)
				{
					vx::RaycastHit hit = processor.Hits()[i];
					if (hit.body != mBody)
					{
						end = ray_cast.PointAlongRay(hit.fraction);

						vx::Colour col = vx::Colour::sYellow;
						mDebugGizmos->DrawAACross(end, &col, 1, 0.3f);
						break;
					}
				}
			}
			mDebugGizmos->DrawLine(ray_cast.origin, end, vx::Colour::sCyan);

		}
		else if (mBody.IsValid() && (mMouseEvent == EClickEvent::Up || mMouseEvent == EClickEvent::None))
		{
			vx::Body& body = mPhysicsWorld->GetBodyManager().GetBody(mBody);
			//might just release key jolt body 
			//body.ApplyImpulse(vx::Vec3(0.0f, 1.0f, 0.0f) * 10.0f * (1.0f/body.GetInverseMass()));
			body.WakeUp(vx::Vec3(0.0f, 1.0f, 0.0f) * 1000.0f);
			mBody = vx::BodyID();
		}
		else
			mBody = vx::BodyID();
	}
}

void Scenario::CreateGroundPlane(float half_size)
{
	VX_ASSERT(mPhysicsWorld, "Physics World is null");

	vx::BodySettings bodies_settings = vx::BodySettings::DefaultStaticConstruct();
	bodies_settings.debug_name = "ground";
	bodies_settings.shape = new vx::PlaneShape(vx::Vec3::Up(), half_size);
	mPhysicsWorld->CreateBody(bodies_settings);
}

void Scenario::CreateBoxStack(const vx::BodySettings& body_settings, 
	const vx::Vec3& counts, const vx::Vec3& half_extent, const vx::Vec3& base_pos)
{
	int countX = counts.X();
	int countY = counts.Y();
	int countZ = counts.Z();

	vx::Vec3 box_size = 2 * half_extent;


	vx::BodySettings settings = body_settings; //cpy

	for (int y = 0; y < countY; ++y)
		for (int x = 0; x < countX; ++x)
			for (int z = 0; z < countZ; ++z)
			{
				float _x = base_pos.X() + x * box_size.X();
				float _y = base_pos.Y() + y * box_size.Y();
				float _z = base_pos.Z() + z * box_size.Z();

				settings.position = vx::Vec3(_x, _y, _z);
				mPhysicsWorld->CreateBody(settings);
			}
}

void Scenario::CreateBoxPyramidStack(const vx::BodySettings& body_settings, int base_width, int base_depth, int height, const vx::Vec3& half_extent, const vx::Vec3& base_pos)
{
	
	vx::BodySettings settings = body_settings; //cpy

	vx::Vec3 box_size = 2 * half_extent;

	float gap = 0.001f;
	for (int y = 0; y < height; ++y)
	{
		int layer_width = base_width - y;
		int layer_depth = base_depth - y;

		//reach top 
		if (layer_width <= 0 || layer_depth <= 0)
			break;

		

		for(int x= 0; x<layer_width;++x)
			for (int z = 0; z < layer_depth; ++z)
			{



				float _x = base_pos.X() + (x - (layer_width-1) * 0.5f) * box_size.X();
				float _y = base_pos.Y() + y * (box_size.Y() + gap);
				float _z = base_pos.Z() + (z - (layer_depth - 1) * 0.5f) * box_size.Z();

				settings.position = vx::Vec3(_x, _y, _z);
				mPhysicsWorld->CreateBody(settings);
			}
	}
}

void Scenario::Create1DBoxPyramidStack(const vx::BodySettings& body_settings, int base_width, int base_depth, int height, const vx::Vec3& half_extent, const vx::Vec3& base_pos, vx::Axis shrink_axis)
{
	vx::BodySettings settings = body_settings; //cpy

	vx::Vec3 box_size = 2 * half_extent;

	float gap = 0.001f;
	gap = 0.0f;
	for (int y = 0; y < height; ++y)
	{
		int layer_width = base_width;
		int layer_depth = base_depth;


		if (shrink_axis == vx::kAxisX)
			layer_width -= y;
		else
			layer_depth -= y;

		//reach top 
		if (layer_width <= 0 || layer_depth <= 0)
			break;



		for (int x = 0; x < layer_width; ++x)
			for (int z = 0; z < layer_depth; ++z)
			{



				float _x = base_pos.X() + (x - (layer_width - 1) * 0.5f) * box_size.X();
				float _y = base_pos.Y() + y * (box_size.Y() + gap);
				float _z = base_pos.Z() + (z - (layer_depth - 1) * 0.5f) * box_size.Z();

				settings.position = vx::Vec3(_x, _y, _z);
				mPhysicsWorld->CreateBody(settings);
			}
	}
}

void Scenario::CreateJenga(vx::BodySettings body_setting, const vx::Vec3& half_extent, int layers, vx::Vec3 base_pos, float gap)
{

	//InterleavePattern pattern = InterleavePattern::JengaPattern();
	////Footprint fp = Footprint::BoxFootprint(half_extent.Swizzle<vx::kAxisX, vx::kAxisY, vx::kAxisX>()); //use the same on x & z the width
	//Footprint fp = Footprint::BoxFootprint(half_extent); //use the same on x & z the width
	//body_setting.shape = new vx::BoxShape(half_extent);
	//CreateInterleavedStructure(body_setting, fp, layers, base_pos, pattern, gap);

	//return;

	//float block_length = 2.5f;
	//float block_height = 0.3f;
	//float block_width = 0.7f;
	//float gap = 0.05f;

	vx::BoxShape* xShape = new vx::BoxShape(half_extent.Swizzle<vx::kAxisZ, vx::kAxisY, vx::kAxisX>());

	vx::BoxShape* zShape = new vx::BoxShape(half_extent);

	float width = (half_extent.X() * 2.0f);
	float height = (half_extent.Y() * 2.0f);

	float spacingX = width + gap;
	float spacingY = height + gap;

	for (int y = 0; y < layers; ++y)
	{
		bool rotate = (y % 2 == 1);

		float yPos = base_pos.Y() + y * spacingY;

		for (int i = 0; i < 3; ++i)
		{
			vx::Vec3 pos = base_pos;

			pos[1] = yPos;

			//float offset = (i - 1) * (block_length + gap - block_width);
			//float offset = (i - 1) * ((2.0f * block_width) + gap);
			float offset = (i - 1) * spacingX;

			if (!rotate)
			{
				body_setting.shape = zShape;
				pos[0] += offset;
			}
			else
			{
				body_setting.shape = xShape;
				pos[2] += offset;
			}

			body_setting.position = pos;
			mPhysicsWorld->CreateBody(body_setting);
		}
	}
}


Scenario::InterleavePattern Scenario::InterleavePattern::JengaPattern()
{
	InterleavePattern pattern;
	/// for jenga 
	/// layer 0 - 0.0
	/// layer 1 - 90.0
	/// layer 2 - 0.0
	pattern.Orientation = [](int y, int i, int count)
	{
		//return vx::Quat::Identity();
		float t = (count <= 1) ? 0.5f : float(i) / float(count - 1);
		float max_angle = vx::DegToRad(45.0f);
		float angle = (t - 0.5f) * 2.0f * max_angle;

		float base = 90.0f;
		angle = vx::DegToRad(base - 30.0f * i);


		float radius = 3.5f;
		 angle = i * ((2.0f * 1.5f) / radius);
		return vx::Quat::FromAxisAngle(vx::Vec3::Up(), angle);

		//return (y % 2 == 0) ? vx::Quat::Identity() : vx::Quat::FromAxisAngle(vx::Vec3::Up(), vx::DegToRad(90.0f));  
	};

	pattern.Position = [](vx::Vec3& base, float height_offset, int y, int i, int count)
		{

			float t = (count <= 1) ? 0.5f : float(i) / float(count - 1);
			float radius = 3.5f;
			float angle = i * ((2.0f * 1.5f) / radius);
			float x = vx::VxCos(angle) * radius;
			float z = vx::VxSin(angle) * radius;
			return base + vx::Vec3(x, y * 1.5f, z); 



			//rotate y 
			//angle = vx::DegToRad(angle);
			//angle = vx::DegToRad(60.0f * i);

			//return (y % 2 == 0) ? vx::Quat::Identity() : vx::Quat::FromAxisAngle(vx::Vec3::Up(), vx::DegToRad(90.0f));
		};

	pattern.Axis = [](int y) {return (y % 2 == 0) ? vx::Axis::X : vx::Axis::Z; };

	pattern.Direction = [](int y)
		{
			return ((y % 2) == 1) ? vx::Vec3(1.0f, 0.0f, 0.0f) : vx::Vec3(0.0f, 0.0f, 1.0f);

			vx::Vec3 base = vx::Vec3(1, 0, 0);
			//float angle = (y % 2 == 0) ? 30.0f : 120.0f;
			float angle = (y % 2 == 0) ? 0.0f : 90.0f;

			//rotate y 
			angle = vx::DegToRad(angle);
			float c = vx::VxCos(angle);
			float s = vx::VxSin(angle);

			vx::Vec3 rot = base;
			rot[0] = base.X() * c + base.Z() * s;
			rot[2] = -base.X() * s + base.Z() * c;

			return rot;
		};

	return pattern;
}



void Scenario::CreateInterleavedStructure(vx::BodySettings body_setting, const Footprint& fp, int layers, vx::Vec3 base_pos, const InterleavePattern& pattern, float gap)
{
	float spacingX = fp.x + gap;
	float spacingY = fp.y + gap;
	float spacingZ = fp.z + gap;


	//body_setting.shape = zShape; assign body so structure creation does not set/modify shape 
	// and support multi shared shape
	vx::Quat oriented_axisZ = vx::Quat::FromAxisAngle(vx::Vec3::Up(), vx::DegToRad(90.0f));

	//for (int y = 0; y < layers; ++y)
	//{
	//	//bool rotate = pattern.RotateLayer(y);

	//	float yPos = base_pos.Y() + y * spacingY;

	//	vx::Axis axis = pattern.Axis(y);

	//	body_setting.orientation = (axis == vx::Axis::Z) ? oriented_axisZ : vx::Quat::Identity();
	//	for (int i = 0; i < 3; ++i)
	//	{
	//		vx::Vec3 pos = base_pos;

	//		pos[1] = yPos;

	//		float offset = (i - 1);

	//		if (axis == vx::Axis::Z)
	//			pos[0] += offset * spacingX;
	//		else
	//			pos[2] += offset * spacingZ;

	//		body_setting.position = pos;
	//		mPhysicsWorld->CreateBody(body_setting);
	//	}
	//}




	layers = 1;
	int layer_count = 2;

	for (int y = 0; y < layers; ++y)
	{
		//bool rotate = pattern.RotateLayer(y);

		float yPos = base_pos.Y() + y * spacingY;

		vx::Axis axis = pattern.Axis(y);

		//body_setting.orientation = (axis == vx::Axis::Z) ? oriented_axisZ : vx::Quat::Identity();
		for (int i = 0; i < layer_count; ++i)
		{
			vx::Vec3 pos = base_pos;

			pos[1] = yPos;

			float offset = (i - 1);

			vx::Vec3 dir = pattern.Direction(y);
			pos += dir * offset;

			//if (axis == vx::Axis::Z)
			//	pos[0] += offset * spacingX;
			//else
			//	pos[2] += offset * spacingZ;

			body_setting.position = pos;

			body_setting.orientation = pattern.Orientation(y, i, 10);
			body_setting.position = pattern.Position(base_pos, spacingY, y, i, layer_count);
			mPhysicsWorld->CreateBody(body_setting);
		}
	}
}

void Scenario::CreateStructure(const vx::BodySettings& body_settings, const StructureConfig& cfg)
{
	vx::Vec3 box_size = 2.0f * cfg.halfExtent;

	int countY = int(cfg.count.Y());

	int width = int(cfg.count.X());
	int depth = int(cfg.count.Z());

	vx::BodySettings settings = body_settings; //cpy

	////vectorise shrink rule 
	//const vx::Vec2 shrink_mask = vx::Vec2(
	//	(cfg.shrinkAxis == EShrinkAxis::X || cfg.shrinkAxis == EShrinkAxis::XZ) ? 1 : 0,
	//	(cfg.shrinkAxis == EShrinkAxis::Z || cfg.shrinkAxis == EShrinkAxis::XZ) ? 1 : 0);

	//const vx::Vec2 countXZ = vx::Vec2(width, depth);

	//for (int y = 0; y < countY; ++y)
	//{
	//	//apply rule 
	//	vx::Vec2 currXZ = countXZ - (shrink_mask * y);
	//	width = currXZ.X();
	//	depth = currXZ.Y();

	//	if (width <= 0 || depth <= 0)
	//		break;



	//	for(int x = 0; x < width; ++x)
	//		for (int z = 0; z < depth; ++z)
	//		{
	//			float _x = cfg.basePos.X() +
	//				(x - (width - 1) * 0.5f) * box_size.X();

	//			float _y = cfg.basePos.Y() + y * box_size.Y();

	//			float _z = cfg.basePos.Z() + 
	//				(z - (depth - 1) * 0.5f) * box_size.Z();

	//			settings.position = vx::Vec3(_x, _y, _z);
	//			mPhysicsWorld->CreateBody(settings);
	//		}
	//}



	for (int y = 0; y < countY; ++y)
	{
		//apply rule 
		vx::Vec2 currXZ = cfg.GetSize ? cfg.GetSize(y) : vx::Vec2(width, depth);
		width = int(currXZ.X());
		depth = int(currXZ.Y());

		if (width <= 0 || depth <= 0)
			break;

		vx::Vec3 layer_offset = cfg.GetOffset ? cfg.GetOffset(y) : vx::Vec3(0);

		for (int x = 0; x < width; ++x)
			for (int z = 0; z < depth; ++z)
			{
				if (cfg.PlaceRule && !cfg.PlaceRule(x, y, z, width, depth))
					continue;


				float _x = cfg.basePos.X() +
					(x - (width - 1) * 0.5f) * box_size.X();

				float _y = cfg.basePos.Y() + y * box_size.Y();

				float _z = cfg.basePos.Z() +
					(z - (depth - 1) * 0.5f) * box_size.Z();

				settings.position = vx::Vec3(_x, _y, _z) + layer_offset;
				mPhysicsWorld->CreateBody(settings);
			}
	}
}

