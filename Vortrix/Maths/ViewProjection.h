#pragma once
#include "Mat44.h"


///Program enforce right hand
#define VX_RENDER_CLIP_SPACE_ZO_BIT (1 << 0)
#define VX_RENDER_CLIP_SPACE_NO_BIT (1 << 1)

#define VX_RENDER_CLIP_SPACE_ZO VX_RENDER_CLIP_SPACE_ZO_BIT
#define VX_RENDER_CLIP_SPACE_NO VX_RENDER_CLIP_SPACE_NO_BIT
#define VX_RENDER_CLIP_SPACE VX_RENDER_CLIP_SPACE_NO

namespace vx
{
	/// Default, this system is right-handed, 
	/// positive X-axis points right, Vec3(1.0f, 0.0f, 0.0f).
	/// positive Y-axis points up, Vec3(0.0f, 1.0f, 0.0f).
	/// positive Z-axis point "out" of screen towards viewer.
	/// i.e scene (forward) is -Vec3::Forward(), Vec3(0.0f, 0.0f, -1.0f).
	/// 
	/// 
	
		/// Creates a view matrix from a basis vectors
	/// @param pos view location (eye)
	/// @param side Screen Right (+X View Space/-X World Space)
	/// @param up Screen Up (+Y View Space)
	/// @param fwd World forward vector (+Z World Space)
	static VX_INLINE Mat44 ViewMatrixFromBasis(const Vec3& pos, const Vec3& right, const Vec3& up, const Vec3& forward)
	{
		//World space ro RH View Space (-Z forward)
		/// forward as World +Z 
		/// in View Space 'forward' to be -Z
		/// 
		Vec3 view_dir = -forward;
		return Mat44(Vec4(right.X(), up.X(), view_dir.X(), 0.0f),
			Vec4(right.Y(), up.Y(), view_dir.Y(), 0.0f),
			Vec4(right.Z(), up.Z(), view_dir.Z(), 0.0f),
			Vec4(-Vec3::Dot(right, pos),
				-Vec3::Dot(up, pos),
				-Vec3::Dot(view_dir, pos),
				1.0f));
	}


		//const Vec3 rt = Vec3::Cross(fwd, up).Normalised(); //bugg??? need to normalise
		//const Vec3 u = Vec3::Cross(rt, fwd);
	/// Build a right handed look at view matrix
	/// 
	/// @param pos Camera position
	/// @param target Point camera is looking at
	/// @param up vector of camera upward vector (assumed normalised)
	static VX_INLINE Mat44 LookAt(const Vec3& pos,
								  const Vec3& target,
								  const Vec3& up)
	{

		//X -> Y -> Z -> X

		/// X = Y x Z
		/// Y = Z x X
		/// Z = X x Y
		const Vec3 z_axis = (target - pos).Normalised();
		const Vec3 x_axis = Vec3::Cross(z_axis, up).Normalised();
		const Vec3 y_axis = Vec3::Cross(x_axis, z_axis);
		
		return ViewMatrixFromBasis(pos, x_axis, y_axis, z_axis);
	}


	/// Create a right handed perspective-view
	/// near and far planes correspond to z normalised device 0 and +1 respectively
	/// 
	/// for right handed (Vulkan rendering volume) Screen (0, 0) Top-Left
	/// 
	/// @param fov Field of view (radians)
	/// @param aspect Aspect ratio of the field of veiw in x direction (ratio of x(width) to y(height).
	/// @param near Distance from view to the near clipping plane
	/// @param far Distance from view to the far clipping plane
	static VX_INLINE Mat44 Perspective_ZO(float fov,
									   float aspect,
									   float zNear,
									   float zFar)
	{
		float h = 1.0f / VxTan(0.5f * fov);
		float w = h / aspect;
		float range = zFar / (zNear - zFar);

		return Mat44(
			Vec4(w, 0.0f, 0.0f, 0.0f),
			Vec4(0.0f, h, 0.0f, 0.0f),
			Vec4(0.0f, 0.0f, range, -1.0f),
			Vec4(0.0f, 0.0f, -range * zNear, 0.0f));
			//Vec4(0.0f, 0.0f, -(zFar * zNear) / (zFar - zNear), 0.0f));
	}

	/// Create a right handed perspective-view
	/// near and far planes correspond to z normalised device -1 and +1 respectively
	/// 
	/// for (OpenGL rendering volume) Screen (0, 0) Bottom-Left
	/// 
	/// @param fov Field of view (radians)
	/// @param aspect Aspect ratio of the field of veiw in x direction (ratio of x(width) to y(height).
	/// @param near Distance from view to the near clipping plane
	/// @param far Distance from view to the far clipping plane
	static VX_INLINE Mat44 Perspective_NO(float fov,
										    float aspect,
										    float zNear,
										    float zFar)
	{
		float h = 1.0f / VxTan(0.5f * fov);
		float w = h / aspect;
		float diff = zFar - zNear;

		return Mat44(
			Vec4(w, 0.0f, 0.0f, 0.0f),
			Vec4(0.0f, h, 0.0f, 0.0f),
			Vec4(0.0f, 0.0f, -(zFar + zNear)/diff, -1.0f),
			Vec4(0.0f, 0.0f, -(2.0f * zFar * zNear)/diff, 0.0f));
	}

	/// Create a right handed perspective-view,
	/// Default near and far is NO correspond to z normalised device 0 and +1 respectively
	/// 
	/// @param fov Field of view (radians)
	/// @param aspect Aspect ratio of the field of veiw in x direction (ratio of x(width) to y(height).
	/// @param near Distance from view to the near clipping plane
	/// @param far Distance from view to the far clipping plane
	static VX_INLINE Mat44 Perspective(float fov,
									   float aspect,
									   float zNear,
									   float zFar)
	{
#if VX_RENDER_CLIP_SPACE == VX_RENDER_CLIP_SPACE_ZO
		return Perspective_ZO(fov, aspect, zNearOffset, zFar);
#else VX_RENDER_CLIP_SPACE == VX_RENDER_CLIP_SPACE_NO
		return Perspective_NO(fov, aspect, zNear, zFar);
#endif // VX_RENDER_CLIP_SPACE == VX_RENDER_CLIP_SPACE_ZO

	}


	/// Create a right handed orthographic-view
	/// near and far planes correspond to z normalised device 0 and +1 respectively
	/// 
	/// for right handed (Vulkan rendering volume) Screen (0, 0) Top-Left
	static VX_INLINE Mat44 Orthographic_ZO(float left,
		float right,
		float bottom,
		float top,
		float zNear,
		float zFar)
	{
		float inv_rt_diff = 1.0f / (right - left);
		float inv_tb_diff = 1.0f / (top - bottom);
		float inv_fn_diff = 1.0f / (zFar - zNear);

		return Mat44(
			Vec4(2.0f * inv_rt_diff, 0.0f, 0.0f, 0.0f),
			Vec4(0.0f, 2.0f * inv_tb_diff, 0.0f, 0.0f),
			Vec4(0.0f, 0.0f, -inv_fn_diff, 0.0f),
			Vec4(-(right + left) * inv_rt_diff,
				-(top + bottom) * inv_tb_diff,
				-zNear * inv_fn_diff,
				1.0f));
	}
	

	/// Create a right handed orthographic-view
	/// near and far planes correspond to z normalised device -1 and +1 respectively
	/// 
	/// for (OpenGL rendering volume) Screen (0, 0) Bottom-Left
	static VX_INLINE Mat44 Orthographic_NO(float left,
										   float right,
										   float bottom,
										   float top,
										   float zNear,
										   float zFar)
	{
		float inv_rt_diff = 1.0f / (right - left);
		float inv_tb_diff = 1.0f / (top - bottom);
		float inv_fn_diff = 1.0f / (zFar - zNear);

		return Mat44(
			Vec4(2.0f * inv_rt_diff, 0.0f, 0.0f, 0.0f),
			Vec4(0.0f, 2.0f * inv_tb_diff, 0.0f, 0.0f),
			Vec4(0.0f, 0.0f, -2.0f * inv_fn_diff, 0.0f), //bug ???? 2.0f * inv_fn_diff > (-2.0f * inv_fn_diff)
			Vec4(-(right + left) * inv_rt_diff,
				-(top + bottom) * inv_tb_diff,
				-(zFar + zNear) * inv_fn_diff,
				1.0f));


	}


	static VX_INLINE Mat44 Orthographic(float left,
		float right,
		float bottom,
		float top,
		float zNear,
		float zFar)
	{
#if VX_RENDER_CLIP_SPACE == VX_RENDER_CLIP_SPACE_ZO
		return Orthographic_ZO(left, right, bottom, top, zNearOffset, zFar);
#else VX_RENDER_CLIP_SPACE == VX_RENDER_CLIP_SPACE_NO
		return Orthographic_NO(left, right, bottom, top, zNear, zFar);
#endif // VX_RENDER_CLIP_SPACE == VX_RENDER_CLIP_SPACE_ZO
	}
}