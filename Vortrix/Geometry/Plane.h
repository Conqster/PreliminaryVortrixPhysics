#pragma once

#include "Vortrix/Maths/Vec3.h"
#include <tuple>

namespace vx
{
	//Implicit Plane
//f = [nx, ny, nz, d] => xyz normals , d => constant
	class Plane
	{
	public:
		Plane() = default;
		explicit Plane(const Vec3& nor, float d = 0.0f) :
			mNormal(nor.Normalised()), mConstant(d) {
		}


		//Creation 
		static Plane CreateFromPointAndNormal(const Vec3& pointA, const Vec3& normal)
		{
			const Vec3 n = normal.Normalised();
			return Plane(n, -n.Dot(pointA));
		}


		//retrive 
		const Vec3 GetNormal() const { return mNormal; }
		void SetNormal(const Vec3& nor) { mNormal = nor; }
		const float GetConstant() const { return mConstant; }
		void SetConstant(float d) { mConstant = d; }

		const std::tuple<Vec3, float> GetNormalAndConstant() const { return { mNormal, mConstant }; }


		//Distance point to plane
		//how far point is from plane +ive above, -ive below
		float SignedDistance(const Vec3& pointA) const { return pointA.Dot(mNormal) + mConstant; }

		void Normalise()
		{
			const float len = mNormal.Length();
			if (len > kEpsilon)
			{
				mNormal /= len;
				mConstant /= len;
			}
		}


		Vec3 ProjectPointOnPlane(const Vec3& pointA) { return pointA - mNormal * SignedDistance(pointA); }
		Vec3 PointOnPlane() { return -mConstant * mNormal; }

		void GetBasis(Vec3& right, Vec3& up, Vec3& forward) const
		{
			forward = mNormal;

			Vec3 tangent = Vec3::Up();
			if (VxAbs(forward.X()) > 0.9f)
				tangent = Vec3::Right();

			right = Vec3::Cross(tangent, forward).Normalised();
			up = Vec3::Cross(forward, right);
		}

	private:
		Vec3 mNormal;
		float mConstant;
	};
} //VPHX namespace