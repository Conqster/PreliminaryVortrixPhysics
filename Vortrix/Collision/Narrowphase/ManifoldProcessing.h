#pragma once


#define VX_MAX_MANIFOLD_CANDIDATES 10

namespace vx{
	struct ManifoldPoint;
	class Vec3;
	namespace Narrowphase {

		void SortContactManifold_Deepest(ManifoldPoint* in_out_pts, int count);

		/// returns counts
		/// ensures that the first four points are the main support 
		/// 
		/// make sure to ignore/clamp data to first four
		void ReduceContactManifoldTo4_Geometric(const ManifoldPoint* in_pts, int count, ManifoldPoint* o_optimal_manifold_pts);

		int ClipPolygonAgainstPlane(const Vec3* poly_verts, const int poly_vert_count, const Vec3& plane_nor, float plane_d, Vec3* clipped_points);

		int ClipFaceToBoxSidePlanes(const Vec3* face_verts, int vert_count,
			const Vec3& planes_center, const Vec3& planes_tangent, const float tangent_extent,
			const Vec3& planes_bitangent, float bitangenet_extent, Vec3* out_verts);

	} /// namespace Narrowphase
} /// namespace vx