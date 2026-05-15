#include "Camera.h"

#include "Vortrix/Core/Logger.h"

#include "Vortrix/Maths/ViewProjection.h"



Camera::Camera(const vx::Vec3& pos, float pitch, float yaw, float move_speed, float rot_speed)
	: mState(pos, pitch, yaw), mProperties({ move_speed, rot_speed }){}

Camera::Camera(const vx::Vec3& pos, const vx::Vec3& forward, const vx::Vec3& up, float move_speed, float rot_speed) : 
	mState(pos, forward, up), mProperties(move_speed, rot_speed) {}

vx::Mat44 Camera::ViewMat()
{
	//Update();
	//return vx::LookAt(mState.position, mState.position + mState.forward, mState.up);
	return vx::ViewMatrixFromBasis(mState.position, GetRight(), mState.up, mState.forward);
	//return vx::ViewMatrixFromBasis(mState.position, GetRight(), mState.up, -mState.forward);
}

vx::Mat44 Camera::ProjMat(float aspect_ratio)
{
	return vx::Perspective(vx::DegToRad(mProperties.fovY), aspect_ratio, mProperties.zNear, mProperties.zFar);
}

void Camera::Translate(vx::Vec3 dir, float dt)
{
	mState.position += dir * mProperties.moveSpeed * dt;
}

void Camera::Rotate(float dx, float dy, float dt)
{
	float d_yaw = dx * mProperties.rotSpeed * dt;
	float d_pitch = dy * mProperties.rotSpeed * dt;
	float pitch = vx::VxClamp(mState.pitch + d_pitch, -89.0f, 89.0f);
	d_pitch = pitch - mState.pitch;
	mState.pitch = pitch;

	vx::Quat q_yaw = vx::Quat::FromAxisAngle(vx::Vec3::Up(), vx::DegToRad(-d_yaw));

	vx::Vec3 right = -mState.orientation.RotateAxisX().Normalised();
	vx::Quat q_pitch = vx::Quat::FromAxisAngle(right, vx::DegToRad(d_pitch));

	mState.orientation = q_yaw * q_pitch * mState.orientation;
	mState.orientation.Normalise();

	mState.forward = mState.orientation.RotateAxisZ();
	mState.up = mState.orientation.RotateAxisY();

}

void Camera::Update()
{
	//if (vx::VxAbs(mState.yaw) < vx::kEpsilon || vx::VxAbs(mState.pitch) < vx::kEpsilon)
	//	return; 

	//vx::Quat q_yaw = vx::Quat::FromAxisAngle(vx::Vec3::Up(), vx::DegToRad(-mState.yaw));

	//vx::Vec3 curr_fwd = q_yaw.RotateAxisZ();
	//vx::Vec3 right = curr_fwd.Cross(vx::Vec3::Up());
	//float len_sq = right.LengthSq();
	//right = (len_sq > 1e-6f) ? right / vx::VxSqrt(len_sq) : vx::Vec3::Right();
	//vx::Quat q_pitch = vx::Quat::FromAxisAngle(right, vx::DegToRad(mState.pitch));

	//mState.forward = q_pitch.Rotate(curr_fwd).Normalised();
	//mState.up = q_pitch.Rotate(q_yaw.RotateAxisY()).Normalised();
}
