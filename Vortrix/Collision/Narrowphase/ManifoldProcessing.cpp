#include "ManifoldProcessing.h"

#include "Vortrix/Core/Assertion.h"
#include "Vortrix/Collision/ContactManifold.h"
#include "Vortrix/Maths/VortrixMaths.h"
#include <array>

namespace vx::Narrowphase {

	void SortContactManifold_Deepest(ManifoldPoint* in_out_pts, int count)
	{
		VX_ASSERT_WARN_VOID(count > 0 && count <= 4,
			"Manifold point in count is out bound 0 or more than 4; only support 4 points at the moment");

		count = VxMin(count, 4);
		/// copy data before overwrite
		std::array<ManifoldPoint, ContactManifold::kMaxPoints> pts;

		int deepest = 0;
		//std::array<float, ContactManifold::kMaxPoints> penetration_depth_sq;
		constexpr float k_min_penetration_sq = 1e-6f; // deal with potential floating point error 
		float max_penetration_sq = k_min_penetration_sq;
		for (int i = 0; i < count; ++i)
		{
			float depth_sq = VxMax(k_min_penetration_sq, (in_out_pts[i].pointB - in_out_pts[i].pointA).LengthSq());
			if (depth_sq > max_penetration_sq)
			{
				deepest = i;
				max_penetration_sq = depth_sq;
			}
			pts[i] = in_out_pts[i];
		}


		/// Fix later to use a while loop 
		/// with levels 0 -> 1 -> 2 -> 3
		/// 0 = deepest
		/// 1 = max distance to deepest
		/// 2 = pt with furthest to SegmentAB (pts lvl0 & 1) 
		/// 3 = pt for max area of ABCD


		//move the deepest to  the start of out
		//std::swap(in_out_pts[0], in_out_pts[deepest]);
		in_out_pts[0] = in_out_pts[deepest];
		//remove -- move to the end
		std::swap(pts[deepest], pts[count - 1]);

		count--;
		if (count <= 0)
			return;


		/// find farthest B from A
		/// B -> farthest from A (max distance)
		int B = 0;
		float best_dist = -1.0f;
		for (int i = 0; i < count; ++i)
		{
			float dist = (pts[i].pointA - in_out_pts[0].pointA).LengthSq();
			if (dist > best_dist)
			{
				best_dist = dist;
				B = i;
			}
		}

		in_out_pts[1] = pts[B];
		/// move to the end
		std::swap(pts[B], pts[count - 1]);

		count--;
		if (count <= 0)
			return;

		/// C -> triangle ABC, (max tri area) farthest from segment AB
		Vec3 AB = in_out_pts[1].pointA - in_out_pts[0].pointA;
		int C = 0;
		best_dist = -1.0f;
		for (int i = 0; i < count; ++i)
		{
			Vec3 A_2_pt = pts[i].pointA - in_out_pts[0].pointA;
			/// Area proportional |AB x AP| area = (|AB x AP|)^2
			float area = A_2_pt.Cross(AB).LengthSq();
			if (area > best_dist)
			{
				best_dist = area;
				C = i;
			}
		}


		in_out_pts[2] = pts[C];
		/// move to the end
		std::swap(pts[C], pts[count - 1]);

		count--;
		if (count <= 0)
			return;

		/// D -> tetrahedron ABCD, (max area) 
		Vec3 ABxAC = AB.Cross(in_out_pts[2].pointA - in_out_pts[0].pointA);
		int D = 0;
		best_dist = -1.0f;
		for (int i = 0; i < count; ++i)
		{
			Vec3 A_2_pt = pts[i].pointA - in_out_pts[0].pointA;
			float vol = VxAbs(A_2_pt.Dot(ABxAC));
			/// Volume proportional |(AB x AC) .AP| area = (|(AB x AC) .AP|)/6 proportional to tetrahedron vol
			if (vol > best_dist)
			{
				best_dist = vol;
				D = i;
			}
		}

		in_out_pts[3] = pts[D];
	}


	void ReduceContactManifoldTo4_Geometric(const ManifoldPoint* in_pts, int count, ManifoldPoint* o_optimal_manifold_pts)
	{
		if (count <= 4)
			return;

		/// required points A, B, C & D
		/// A -> deepest pt (max penetration)
		/// B -> farthest from A (max distance)
		/// C -> triangle ABC, (max tri area) farthest from segment AB
		/// D -> tetrahedron ABCD, (max area) 
		/// 

		count = VxMin(count, VX_MAX_MANIFOLD_CANDIDATES);

		/// TO-DO(Jay): change heap allocation to stack
		//std::vector<ManifoldPoint> pts(count);
		ManifoldPoint pts[VX_MAX_MANIFOLD_CANDIDATES];

		//deepest peneration
		//int deepest = 0;
		//float max_penetration = in_pts[0].peneration;
		//for (int i = 0; i < count; ++i)
		//{
		//	if (in_pts[i].peneration > max_penetration)
		//	{
		//		deepest = i;
		//		max_penetration = in_pts[i].peneration;
		//	}
		//	pts[i] = in_pts[i];
		//}

		int deepest = 0;
		constexpr float k_min_penetration_sq = 1e-6f;
		//float penetration_depth_sq[VX_MAX_MANIFOLD_CANDIDATES];
		float max_penetration_sq = k_min_penetration_sq;
		for (int i = 0; i < count; ++i)
		{
			float depth_sq = VxMax(k_min_penetration_sq, (in_pts[i].pointB - in_pts[i].pointA).LengthSq());
			if (depth_sq > max_penetration_sq)
			{
				deepest = i;
				max_penetration_sq = depth_sq;
			}
			pts[i] = in_pts[i];
		}


		//std::array<ManifoldPoint, 4> optimal_pts;
		o_optimal_manifold_pts[0] = in_pts[deepest];
		//remove -- move to the end
		std::swap(pts[deepest], pts[count - 1]);
		count--;


		/// find farthest B from A
		/// B -> farthest from A (max distance)
		int B = 0;
		float best_dist = -1.0f;
		for (int i = 0; i < count; ++i)
		{
			float dist = (pts[i].pointA - o_optimal_manifold_pts[0].pointA).LengthSq();
			if (dist > best_dist)
			{
				best_dist = dist;
				B = i;
			}
		}

		o_optimal_manifold_pts[1] = pts[B];
		/// move to the end
		std::swap(pts[B], pts[count - 1]);
		count--;

		/// C -> triangle ABC, (max tri area) farthest from segment AB
		Vec3 AB = o_optimal_manifold_pts[1].pointA - o_optimal_manifold_pts[0].pointA;
		int C = 0;
		best_dist = -1.0f;
		for (int i = 0; i < count; ++i)
		{
			Vec3 A_2_pt = pts[i].pointA - o_optimal_manifold_pts[0].pointA;
			/// Area proportional |AB x AP| area = (|AB x AP|)^2
			float area = A_2_pt.Cross(AB).LengthSq();
			if (area > best_dist)
			{
				best_dist = area;
				C = i;
			}
		}

		o_optimal_manifold_pts[2] = pts[C];
		/// move to the end
		std::swap(pts[C], pts[count - 1]);
		count--;


		/// D -> tetrahedron ABCD, (max area) 
		Vec3 ABxAC = AB.Cross(o_optimal_manifold_pts[2].pointA - o_optimal_manifold_pts[0].pointA);
		int D = 0;
		best_dist = -1.0f;
		for (int i = 0; i < count; ++i)
		{
			Vec3 A_2_pt = pts[i].pointA - o_optimal_manifold_pts[0].pointA;
			float vol = VxAbs(A_2_pt.Dot(ABxAC));
			/// Volume proportional |(AB x AC) .AP| area = (|(AB x AC) .AP|)/6 proportional to tetrahedron vol
			if (vol > best_dist)
			{
				best_dist = vol;
				D = i;
			}
		}

		o_optimal_manifold_pts[3] = pts[D];
	}

	int ClipPolygonAgainstPlane(const Vec3* poly_verts, const int poly_vert_count, const Vec3& plane_nor, float plane_d, Vec3* clipped_points)
	{
		int clipped_count = 0;

		for (int i = 0; i < poly_vert_count; ++i)
		{
			Vec3 v1 = poly_verts[i];
			Vec3 v2 = poly_verts[(i + 1) % poly_vert_count];

			float d1 = plane_nor.Dot(v1) - plane_d;
			float d2 = plane_nor.Dot(v2) - plane_d;

			if (d1 <= 0 && d2 <= 0) //both inside
				clipped_points[clipped_count++] = v2;
			else if (d1 <= 0 && d2 > 0) //v1 inside v2 going out
			{
				float t = d1 / (d1 - d2);
				clipped_points[clipped_count++] = v1 + t * (v2 - v1);
			}
			else if (d1 > 0 && d2 <= 0) //v1 outside, v2 coming in
			{
				float t = d1 / (d1 - d2);
				clipped_points[clipped_count++] = v1 + t * (v2 - v1);
				clipped_points[clipped_count++] = v2;
			}
		}
		return clipped_count;
	}

	int ClipFaceToBoxSidePlanes(const Vec3* face_verts, int vert_count, const Vec3& planes_center, const Vec3& planes_tangent, const float tangent_extent, const Vec3& planes_bitangent, float bitangenet_extent, Vec3* out_verts)
	{

		float dot_T = planes_center.Dot(planes_tangent);
		float dot_B = planes_center.Dot(planes_bitangent);

		std::array<Vec3, 8> buffer;

		//tangent +ive
		int count = ClipPolygonAgainstPlane(face_verts, vert_count,
			planes_tangent, dot_T + tangent_extent, buffer.data());
		if (count < 3) return 0;

		//tangent -ive
		count = ClipPolygonAgainstPlane(buffer.data(), count,
			-planes_tangent, -dot_T + tangent_extent, out_verts);
		if (count < 3) return 0;

		//bitangent +ive
		count = ClipPolygonAgainstPlane(out_verts, count,
			planes_bitangent, dot_B + bitangenet_extent, buffer.data());
		if (count < 3) return 0;

		//bitangent -ive
		count = ClipPolygonAgainstPlane(buffer.data(), count,
			-planes_bitangent, -dot_B + bitangenet_extent, out_verts);

		return count;
	}

} //vx::Narrowphase