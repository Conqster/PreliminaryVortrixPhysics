#include "BoxShape.h"
#include "Vortrix/Geometry/AABB.h"

namespace vx {
	AABB BoxShape::GetLocalBounds() const
	{
		return { -mHalfExtent, mHalfExtent };
	}
	std::array<Vec3, 8> BoxShape::GetCorners() const
	{
		AABB bounds = GetLocalBounds();

		return {
		   Vec3(bounds.mMin.X(), bounds.mMin.Y(), bounds.mMin.Z()),
		   Vec3(bounds.mMax.X(), bounds.mMin.Y(), bounds.mMin.Z()),
		   Vec3(bounds.mMax.X(), bounds.mMax.Y(), bounds.mMin.Z()),
		   Vec3(bounds.mMin.X(), bounds.mMax.Y(), bounds.mMin.Z()),
		   Vec3(bounds.mMin.X(), bounds.mMin.Y(), bounds.mMax.Z()),
		   Vec3(bounds.mMax.X(), bounds.mMin.Y(), bounds.mMax.Z()),
		   Vec3(bounds.mMax.X(), bounds.mMax.Y(), bounds.mMax.Z()),
		   Vec3(bounds.mMin.X(), bounds.mMax.Y(), bounds.mMax.Z())
		};
	}
	std::array<Vec3, 8> BoxShape::GetCornersWS(const Mat44& in_transform) const
	{
		AABB bounds = GetLocalBounds();

		const Vec3 axis_x = in_transform.GetColumn(0);
		const Vec3 axis_y = in_transform.GetColumn(1);
		const Vec3 axis_z = in_transform.GetColumn(2);
		const Vec3 center = in_transform.GetColumn(3);

		const Vec3 ex = axis_x * mHalfExtent.X();
		const Vec3 ey = axis_y * mHalfExtent.Y();
		const Vec3 ez = axis_z * mHalfExtent.Z();


		return{
			center + (-ex - ey - ez),
			center + (ex - ey - ez),
			center + (-ex + ey - ez),
			center + (-ex + ey - ez),

			center + (ex - ey + ez),
			center + (-ex - ey + ez),
			center + (-ex + ey + ez),
			center + (ex + ey + ez)
		};
	}
	void BoxShape::GetSupportingFaceVertices(const Vec3& direction, const Vec3& box_min, const Vec3& box_max, std::array<Vec3, 4>& out_vertices)
	{
		int axis = static_cast<int>(direction.Abs().MaxAxis());
		bool postive = direction[axis] >= 0.0f;

		switch (axis)
		{
		case 0://Vec3::AxisXIndex():

			if (postive)
			{
				out_vertices = {
					Vec3(box_max.X(), box_min.Y(), box_min.Z()),
					Vec3(box_max.X(), box_max.Y(), box_min.Z()),
					Vec3(box_max.X(), box_max.Y(), box_max.Z()),
					Vec3(box_max.X(), box_min.Y(), box_max.Z())
				};
			}
			else
			{
				out_vertices = {
					Vec3(box_min.X(), box_min.Y(), box_min.Z()),
					Vec3(box_min.X(), box_min.Y(), box_max.Z()),
					Vec3(box_min.X(), box_max.Y(), box_max.Z()),
					Vec3(box_min.X(), box_max.Y(), box_min.Z())
				};
			}
			break;

		case 1://Vec3::YAxisIndex():

			if (postive)
			{
				out_vertices = {
					Vec3(box_min.X(), box_max.Y(), box_min.Z()),
					Vec3(box_min.X(), box_max.Y(), box_max.Z()),
					Vec3(box_max.X(), box_max.Y(), box_max.Z()),
					Vec3(box_max.X(), box_max.Y(), box_min.Z())
				};
			}
			else
			{
				out_vertices = {
					Vec3(box_min.X(), box_min.Y(), box_min.Z()),
					Vec3(box_max.X(), box_min.Y(), box_min.Z()),
					Vec3(box_max.X(), box_min.Y(), box_max.Z()),
					Vec3(box_min.X(), box_min.Y(), box_max.Z())
				};
			}
			break;

		case 2://Vec3::AxisIndexZ():

			if (postive)
			{
				out_vertices = {
					Vec3(box_min.X(), box_min.Y(), box_max.Z()),
					Vec3(box_max.X(), box_min.Y(), box_max.Z()),
					Vec3(box_max.X(), box_max.Y(), box_max.Z()),
					Vec3(box_min.X(), box_max.Y(), box_max.Z())
				};
			}
			else
			{
				out_vertices = {
					Vec3(box_min.X(), box_min.Y(), box_min.Z()),
					Vec3(box_min.X(), box_max.Y(), box_min.Z()),
					Vec3(box_max.X(), box_max.Y(), box_min.Z()),
					Vec3(box_max.X(), box_min.Y(), box_min.Z())
				};
			}
			break;
		}
	}
	std::tuple<Vec3, Vec3> BoxShape::GetMinMaxOrientedWS(const Mat44& in_transform) const
	{
		Vec3 center = in_transform.GetTranslation();

		Vec3 local_min = -mHalfExtent;
		Vec3 local_max = mHalfExtent;

		Vec3 temp_min = in_transform.GetTranslation();
		Vec3 temp_max = temp_min;
		for (uint i = 0; i < 3; i++)
		{
			Vec3 column = in_transform.GetColumn(i);

			Vec3 a = column * local_min[i];
			Vec3 b = column * local_max[i];

			temp_min += Vec3::Min(a, b);
			temp_max += Vec3::Max(a, b);
		}
		return { temp_min, temp_max };
	}

	std::array<Vec3, 4> BoxShape::GetSupportingFaceVerticesOBB(const Mat44& in_transform, const Vec3& direction) const
	{
		//direction projected onto OBB axes
		Vec3 local_dir = in_transform.Multiply3x3Transposed(direction);
		std::array<Vec3, 4> supporting_vertices = GetSupportingFaceVertices(local_dir);

		Vec3 axis_x = in_transform.GetColumn(0);
		Vec3 axis_y = in_transform.GetColumn(1);
		Vec3 axis_z = in_transform.GetColumn(2);
		Vec3 center = in_transform.GetColumn(3);

		std::array<Vec3, 4> vert;
		//transform
		for (int i = 0; i < 4; i++)
		{
			const Vec3& p = supporting_vertices[i];
			vert[i] = center + axis_x * p.X()
				+ axis_y * p.Y() + axis_z * p.Z();
		}
		return vert;
	}
	Vec3 BoxShape::GetSupportWS(const Mat44& in_transform, const Vec3& dir) const
	{
#define PRECISION 0
#if PRECISION
		Vec3 local_dir = mTransform.Multiply3x3Transposed(dir);
		Vec3 local_support = -local_dir.Sign() * mHalfExtent;
		return mTransform.Transform(local_support);
#else
		const Vec3 x = in_transform.GetAxisX();
		const Vec3 y = in_transform.GetAxisY();
		const Vec3 z = in_transform.GetAxisZ();

		return in_transform.GetTranslation() + x * (dir.Dot(x) >= 0.0f ? mHalfExtent.X() : -mHalfExtent.X()) +
			y * (dir.Dot(y) >= 0.0f ? mHalfExtent.Y() : -mHalfExtent.Y()) +
			z * (dir.Dot(z) >= 0.0f ? mHalfExtent.Z() : -mHalfExtent.Z());
#endif // PRECISION
	}
	Vec3 BoxShape::GetEdgeCenter(const Mat44& in_transform, int edge_idx, const Vec3& normal) const
	{
		Vec3 local_n = in_transform.Multiply3x3Transposed(normal);
		//Vec3 e = -local_n.Sign() * mHalfExtent;
		//frame - frame jitter fix
		const float eps = 1e-4f;
		Vec3 e = Vec3(
			local_n.X() >= eps ? mHalfExtent.X() : -mHalfExtent.X(),
			local_n.Y() >= eps ? mHalfExtent.Y() : -mHalfExtent.Y(),
			local_n.Z() >= eps ? mHalfExtent.Z() : -mHalfExtent.Z());

		e[edge_idx] = 0.0f;
		return in_transform.Transform(e);
	}
	MassProperties BoxShape::GetMassProperties() const
	{
		///inretia tensor 
		/// I = inertia scalar
		/// Ix, Iy, Iz  = scalar on x, y, z resp
		/// 
		/// frac 1/12
		/// m mass
		/// 
		/// w weight (x)
		/// h height (y)
		/// d depth (z)
		/// 
		/// Ix = frac * m (h^2 + d^2)
		/// Iy = frac * m (w^2 + d^2)
		/// Iz = frac * m (w^2 + h^2)
		/// 
		/// mat33 diagonal = Ix, Iy, Iz
		/// 
		Vec3 he2 = 2 * mHalfExtent;
		const float mass = he2.X() * he2.Y() * he2.Z() * mDensity;
		constexpr float frac = 1.0f / 12.0f;

		he2 *= he2;

		float ww = he2.X();
		float hh = he2.Y();
		float dd = he2.Z();


		MassProperties mp;
		mp.mass = mass;
		mp.inertialTensorDiagonal = Float3((frac * mass * (hh + dd)),
			(frac * mass * (ww + dd)),
			(frac * mass * (ww + hh)));

		return mp;
	}
	Float3 BoxShape::ComputeInertiaTensorDiagonal(float mass) const
	{
		Vec3 he2 = 2 * mHalfExtent;

		float ww = he2.X();
		float hh = he2.Y();
		float dd = he2.Z();

		return Float3(((1.0f / 12.0f) * mass * (hh + dd)),
			((1.0f / 12.0f) * mass * (ww + dd)),
			((1.0f / 12.0f) * mass * (ww + hh)));
	}
} //namespace vx