#pragma once

#include "Core.h"
//#include "Mat44.h"
//#include "Vec4.h"
//#include "Vec3.h"

namespace vx
{
	/*
	* Quat q
	* Quaternion class for representing 3D rotations.
	* 
	* Quaternion are 4D vectors that encode 
	* 3 degrees of rotational freedom (DOF) in 3D space.
	* 
	* When normalised (|q| = 1), a quaternion repsent vaild rotation.
	* 
	* Layout: 
	* [x, y, z, w]
	* - [x, y, z]: Imaginary vector (rotation axis * sin(angle/2)) 
	* - [w]: Real scalar (cos(angle/2))
	* 
	* Conventions:
	*	- Right-handed coordinate syste,
	*	- Quat -> Mat44 (Column-major)
	* 
	* Internally stored as a Vec4.
	*/
	class alignas(16) Quat
	{
	public:
		Quat() = default;
		Quat(float x, float y, float z, float w) : mValue(x, y, z, w){}
		Quat(const Vec3& imaginary, float w) : mValue(imaginary, w){}
		Quat(const Vec4& component) : mValue(component){}

		/// @return imaginary x component value
		VX_INLINE float X() const { return mValue.X(); }
		/// @return imaginary y component value
		VX_INLINE float Y() const { return mValue.Y(); }
		/// @return imaginary z component value
		VX_INLINE float Z() const { return mValue.Z(); }
		/// @return real w component value
		VX_INLINE float W() const { return mValue.W(); }

		/// @return imaginary (vector) part
		VX_INLINE Vec3 Imaginary() const { return Vec3(mValue); }
		/// @return full quaternion as Vec4 (x, y, z, w)
		VX_INLINE Vec4 XYZW() const { return mValue; }

		/// Identity quaternion (no rotation)
		/// @return [0, 0, 0, 1]
		static VX_INLINE Quat Identity() { return Quat(0.0f, 0.0f, 0.0f, 1.0f); }
		/// Create quaternion from axis-angle representation
		/// @param axis (assume normalised rotation axis).
		/// @param angle Rotation angle in radians.
		/// @return quat from axis-angle
		static VX_INLINE Quat FromAxisAngle(const Vec3& axis, float angle);
		/// Set quaternion from axis-angle.
		/// @param axis (assume normalised rotation axis).
		/// @param angle Rotation angle in radians.
		VX_INLINE void SetAxisAngle(const Vec3& axis, float angle);
		/// Extract axis-angle representation
		/// Assume this quat q, is Unit quaternion (Normalised, |q| = 1) .
		/// @param o_axis Output rotation axis (normalise)
		/// @param o_angle Output rotation angle in radians
		VX_INLINE void GetAxisAngle(Vec3& o_axis, float& o_angle);

		/// Normlise this vector in place
		VX_INLINE void Normalise();
		/// Normlise this quaternion
		/// @return normalise copy of this 
		/// of length 1.
		VX_INLINE Quat Normalised() const;

		/// Dot product 
		VX_INLINE float Dot(const Quat& rhs) const { return Vec4::Dot(mValue, rhs.mValue); }
		/// Check if quaternion is unit-length (normalised, |q| = 1)
		VX_INLINE bool IsUnitQuat(float tolerance = 1e-4f) const;

		//@return the squared length (magnitude) of the quaternion
		VX_INLINE float LengthSq() const { return mValue.LengthSq(); }
		///@return the length (magnitude) of the quaternion
		VX_INLINE float Length() const { return mValue.Length(); }

		/// Conjugate quaternion
		/// @return Negated imaginary part of quarternion
		VX_INLINE Quat Conjugated() const { return Quat(mValue.FlipSign<-1, -1, -1, 1>()); }
		/// Inverse quaternion
		/// for unit quaternion, inverse == conjugate
		VX_INLINE Quat Inversed() const;
		/// so as forward(world) = q.Rotate(local) + t
		/// and inverse(local) = q.InverseRotate(world - t)
		/// Rotates a vector by this unit quaternion
		/// 
		/// Compute:
		///		v' = q * v * q^-1
		/// 
		/// Optimised Rodrigues formula :
		/// v'= v + 2 * w * (q.xyz x v) + 2 * (q.xyz x (q.xyz x v))
		/// v' = v' = v + wt + (q.xyz x t)
		///		t = 2(q.xyz x v)
		/// 
		/// Quaternion is assume (|q| = 1)
		/// @param vec Vector to rotate
		VX_INLINE Vec3 Rotate(const Vec3& vec) const;
		/// Rotate vector by inverse quaternion
		/// Quaternion is assume (|q| = 1)
		/// Equivalent to transforming into local space
		VX_INLINE Vec3 InverseRotate(const Vec3& vec) const;
		/// Rotate vector using full quaternion multiplication
		/// Quaternion is assume (|q| = 1)
		/// Slower but mathematically explicit
		VX_INLINE Vec3 RotateSlow(const Vec3& vec) const;
		/// Inverse rotation using full quaternion multiplucation
		/// Quaternion is assume (|q| = 1)
		VX_INLINE Vec3 InverseRotateSlow(const Vec3& vec) const;
		/// Rotate world X axis by quaternion
		/// Quaternion is assume (|q| = 1)
		VX_INLINE Vec3 RotateAxisX() const;
		/// Rotate world Y axis by quaternion
		/// Quaternion is assume (|q| = 1)
		VX_INLINE Vec3 RotateAxisY() const;
		/// Rotate world Z axis by quaternion
		/// Quaternion is assume (|q| = 1)
		VX_INLINE Vec3 RotateAxisZ() const;
		/// Rotate world X axis and apply scale
		/// Quaternion is assume (|q| = 1)
		VX_INLINE Vec3 RotateScaledAxisX(float scale) const;
		/// Rotate world Y axis and apply scale
		/// Quaternion is assume (|q| = 1)
		VX_INLINE Vec3 RotateScaledAxisY(float scale) const;
		/// Rotate world Z axis and apply scale
		/// Quaternion is assume (|q| = 1)
		VX_INLINE Vec3 RotateScaledAxisZ(float scale) const;
		/// Rotate and scale all basis axes
		///
		/// Produces scaled orientation axes suitable (rotation/&tranform matrix)
		/// Quaternion is assume (|q| = 1)
		/// 
		/// @param scale Per-axis scale
		/// @param out_x Output X axis
		/// @param out_y Output Y axis
		/// @param out_z Output Z axis
		VX_INLINE void RotateScaledAxes(const Vec3& scale, Vec3& out_x, Vec3& out_y, Vec3& out_z);
		/// Convert quaternion to rotation matrix
		/// Quaternion is assume (|q| = 1)
		/// @return 4x4 column-major rotation matrix
		VX_INLINE Mat44 GetRotationMat44();

		/// Quaternion multipluication (componsition)
		VX_INLINE Quat operator*(const Quat& rhs) const;
		/// Quaternion multipluication (in place)
		VX_INLINE Quat operator*=(const Quat& rhs);
		/// Rotate vector (same as Rotate(const Vec3))
		VX_INLINE Vec3 operator*(const Vec3& vec) const { return Rotate(vec); }
		/// Scalar division
		VX_INLINE Quat operator/(float rhs) const { return Quat(mValue / rhs); }
		/// Negate all component
		VX_INLINE Quat operator -()const { return Quat(-mValue); }


		///NEW 
		VX_INLINE Quat operator*(float rhs) const
		{
			return Quat(mValue * rhs);
		}

		VX_INLINE friend Quat operator*(float rhs, const Quat& q)
		{
			return Quat(q.mValue * rhs);
		}

		VX_INLINE Quat operator*=(float rhs)
		{
			mValue *= rhs;
			return *this;
		}

		VX_INLINE void operator+=(const Quat& rhs)
		{
			mValue += rhs.mValue;
		}

		VX_INLINE std::string ToString() const
		{
			return mValue.ToString();
		}

		VX_INLINE friend std::ostream& operator<<(std::ostream& os, const Quat& q)
		{
			os << "Quat(" << q.X() << ", " << q.Y() << ", " << q.Z() << ", " << q.W() << ")";
			return os;
		}



		/// NEW NEW ???
		static VX_INLINE Quat FromEulerAngle(float pitch, float yaw, float roll)
		{
			float half_pitch = 0.5f * pitch;
			float half_yaw = 0.5f * yaw;
			float half_roll = 0.5f * roll;

			float cp = VxCos(half_pitch);
			float sp = VxSin(half_pitch);
			float cy = VxCos(half_yaw);
			float sy = VxSin(half_yaw);
			float cr = VxCos(half_roll);
			float sr = VxSin(half_roll);

			//Q = qx(pitch) * qy(yaw) * qz(roll)
			//return Quat(cr * sp * cy - sr * cp * sy, //x
			//			cr * cp * sy + sr * sp * cy, //y
			//			sr * cp * cy - cr * sp * sy, //z
			//			cr * cp * cy + sr * sp * sy); //w
			

			Quat q(sp * cy * cr + cp * sy * sr, //x
				   cp * sy * cr - sp * cy * sr, //y
				   cp * cy * sr + sp * sy * cr, //z
				   cp * cy * cr - sp * sy * sr); //w
			q.Normalise();
			return q;
		}

		/// Create quat from euler angle
		/// @param angles representing rotation in euler
		/// pitch yaw roll
		/// Rotation order is X then Y then Z (rot_x * rot_y * rot_z)
		static VX_INLINE Quat FromEulerAngle(const Vec3& angles)
		{
			return FromEulerAngle(angles.X(), angles.Y(), angles.Z());
		}


		VX_INLINE Vec3 GetEulerAngles() const
		{
			float sinp = 2.0f * W() * X() + Y() * Z();
			float cosp = 1.0f - 2.0f * (X() * X() + Y() * Y());
			float pitch = VxAtan2(sinp, cosp);

			float siny = 2.0f * (W() * Y() - Z() * X());
			siny = VxClamp(siny, -1.0f, 1.0f);
			float yaw = VxAsin(siny);

			float sinr = 2.0f * (W() * Z() + X() * Y());
			float cosr = 1.0f - 2.0f * (Y() * Y() + Z() * Z());
			float roll = VxAtan2(sinr, cosr);

			return Vec3(pitch, yaw, roll);

			/*float pitch, yaw, roll;

			float siny = 2.0f * (W() * Y() - Z() * X());
			siny = VxClamp(siny, -1.0f, 1.0f);
			yaw = VxAsin(siny);

			if (VxAbs(siny) < 0.9999f)
			{
				pitch = VxAtan2(
					2.0f * (W() * X() + Y() * Z()),
					1.0f - 2.0f * (X() * X() + Y() * Y())
				);

				roll = VxAtan2(
					2.0f * (W() * Z() + X() * Y()),
					1.0f - 2.0f * (Y() * Y() + Z() * Z())
				);
			}
			else
			{
				pitch = 0.0f;
				roll = VxAtan2(
					-2.0f * (X() * Z() - W() * Y()),
					1.0f - 2.0f * (Y() * Y() + Z() * Z())
				);
			}

			return Vec3(pitch, yaw, roll);*/
		}


		static Quat LookRotation(const Vec3& forward, const Vec3& up)
		{
		//	if(VxAbs(forward.Dot(up))>0.999f)
			
			Vec3 f = forward.Normalised();
			Vec3 u = up;
			if (VxAbs(f.Dot(u)) > 0.999f)
				u = f.NormalisedPerpendicular();



			Vec3 r = u.Cross(f).Normalised();
			Vec3 t = f.Cross(r).Normalised();

			Mat44 m(1.0f);

			m.SetColumn3(0, vx::Vec3(r.X(), r.Y(), r.Z()));
			m.SetColumn3(1, vx::Vec3(t.X(), t.Y(), t.Z()));
			m.SetColumn3(2, vx::Vec3(f.X(), f.Y(), f.Z()));

			return m.GetRotationQuat().Normalised();

			//float m00 = r.X(), m01 = t.X(), m02 = forward.X();
			//float m10 = r.Y(), m11 = t.Y(), m12 = forward.Y();
			//float m20 = r.Z(), m21 = t.Z(), m22 = forward.Z();

			//float trace = m00 + m11 + m22;
			//
			//float x, y, z, w;
			//if (trace > 0.0f)
			//{
			//	float s = VxSqrt(1.0f + m00 - m11 - m22) * 2.0f;
			//	w = (m21 - m12) / s;
			//	x = 0.25f * s;
			//	y = (m02 + m20) / s;
			//	z = (m10 + m01) / s;
			//}
			//else if (m00 > m11 && m00 > m22)
			//{
			//	float s = VxSqrt(trace + 1.0f) * 2.0f;
			//	w = 0.25f * s;
			//	x = (m21 - m12) / s;
			//	y = (m02 - m20) / s;
			//	z = (m10 - m01) / s;
			//}
			//else if (m11 > m22)
			//{
			//	float s = VxSqrt(1.0f + m11 - m00 - m22) * 2.0f;
			//	w = (m02 - m20) / s;
			//	x = (m10 + m01) / s;
			//	y = 0.25f * s;
			//	z = (m12 - m21) / s;
			//}
			//else
			//{
			//	float s = VxSqrt(1.0f + m22 - m00 - m11) * 2.0f;
			//	w = (m10-m01) / s;
			//	x = (m02 + m20) / s;
			//	y = (m12 + m21) / s;
			//	z = 0.25f * s;
			//}

			//Quat q(x, y, z, w);
			//return q.Normalised();
		}

	private:
		Vec4 mValue;
	};
}
#include "Quat.inl"