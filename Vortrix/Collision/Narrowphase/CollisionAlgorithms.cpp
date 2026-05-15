#include "CollisionAlgorithms.h"

#include "ManifoldProcessing.h"


namespace vx::Narrowphase {

#pragma region Helper Functions

	template<typename A, typename B>
	static std::tuple<const A*, const B*> GetShapesType(const Shape* a, const Shape* b)
	{
		const A* shape_a = static_cast<const A*>(a);
		const B* shape_b = static_cast<const B*>(b);

		if (!shape_a && a)
			VX_LOG_WARN("Trying to dynamic cast Shape: ", a->GetShapeTypeName(), " to Shape: ", A::GetDebugName());// , GetEShapeTypeName(A().GetType()))

		if (!shape_b && b)
			VX_LOG_WARN("Trying to dynamic cast Shape: ", a->GetShapeTypeName(), " to Shape: ", A::GetDebugName());

		return std::tuple(shape_a, shape_b);
	}

	VX_INLINE void SwapContactOrientation(ContactManifold& in_out_m)
	{
		in_out_m.normal = -in_out_m.normal;
		for (int i = 0; i < in_out_m.numManifoldPoints; ++i)
		{
			auto& mp = in_out_m.points[i];
			std::swap(mp.pointA, mp.pointB);
		}
	}

	VX_INLINE void SwapContactManifoldPoints(ManifoldPoint* in_out_m, int count)
	{
		for (int i = 0; i < count; ++i)
		{
			auto& mp = in_out_m[i];
			std::swap(mp.pointA, mp.pointB);
		}
	}

#pragma endregion



	bool SphereVsSphere(const Shape* in_sphere_a, const Vec3& in_posA, const Quat& in_orientationA, const Shape* in_sphere_b, const Vec3& in_posB, const Quat& in_orientationB, ContactManifold& o_manifold)
	{
		const SphereShape* sphere_a = static_cast<const SphereShape*>(in_sphere_a);
		const SphereShape* sphere_b = static_cast<const SphereShape*>(in_sphere_b);

		VX_ASSERT_WARN_RETURN(sphere_a && sphere_b, false, "either both sphere does not exist, cast failed");

		const Vec3 d_ab = in_posB - in_posA;
		const float dist_sq = d_ab.LengthSq();

		float ra = sphere_a->GetRadius();
		float rb = sphere_b->GetRadius();
		const float sum_radius = ra + rb;

		if (dist_sq > sum_radius * sum_radius) return false;

		Vec3 nor;
		const float dist = VxSqrt(dist_sq);
		if (dist_sq < 1e-6f)
			nor = Vec3::Up();
		else
			nor = d_ab / dist;
	

		o_manifold.normal = nor;
		o_manifold.numManifoldPoints = 1;

		o_manifold.points[0] = {
			in_posA + nor * ra,
			sum_radius - dist, //dummy penetration remove later
			in_posB - nor * rb
		};
		return true;
	}

	bool SphereVsPlane(const Shape* in_sphere, const Vec3& in_posA, const Quat& in_orientationA, const Shape* in_plane, const Vec3& in_posB, const Quat& in_orientationB, ContactManifold& o_manifold)
	{
		const SphereShape* sphere_shape = nullptr;
		const PlaneShape* plane_shape = nullptr;

		std::tie(sphere_shape, plane_shape) = GetShapesType<SphereShape, PlaneShape>(in_sphere, in_plane);
		VX_ASSERT_WARN_RETURN(sphere_shape && plane_shape, false, "either sphere shape or plane shape does not exist, cast failed");

		const Mat44 plane_transform = Mat44::RotationTranslation(in_orientationB, in_posB);
		const Vec3 sphere_pos = in_posA;
		const float radius = sphere_shape->GetRadius();
		const Vec3& plane_n = plane_shape->GetNormal();

		Vec3 local_sphere = plane_transform.TransformInverse(sphere_pos);
		float dist = plane_n.Dot(local_sphere) - plane_shape->GetOffset();
		if (dist >= radius) return false;

		Vec3 local_pt_on_plane = local_sphere - plane_n * dist;
		const Vec3& he = in_plane->GetHalfExtents();
		if (VxAbs(local_pt_on_plane.X()) > he.X() ||
			VxAbs(local_pt_on_plane.Y()) > he.Y() ||
			VxAbs(local_pt_on_plane.Z()) > he.Z()) return false;


		Vec3 world_nor = plane_transform.Multiply3x3(-plane_n).Normalised();

		Vec3 world_pt_on_plane = plane_transform.Transform(local_pt_on_plane);

		float penetration = radius - dist;

		o_manifold.normal = world_nor;
		o_manifold.numManifoldPoints = 1;
		o_manifold.points[0] = {
			sphere_pos + world_nor * radius,
			penetration,
			world_pt_on_plane
		};
		return true;
	}


	bool CapsuleVsSphere(const Shape* in_capsule, const Vec3& in_posA, const Quat& in_orientationA, const Shape* in_sphere, const Vec3& in_posB, const Quat& in_orientationB, ContactManifold& o_manifold)
	{
		const CapsuleShape* capsule = nullptr;
		const SphereShape* sphere = nullptr;

		std::tie(capsule, sphere) = GetShapesType<CapsuleShape, SphereShape>(in_capsule, in_sphere);
		VX_ASSERT_WARN_RETURN(capsule && sphere, false, "either capsule or sphere does not exist, cast failed");

		const Mat44 capsule_transform = Mat44::RotationTranslation(in_orientationA, in_posA);
		Vec3 axis = capsule_transform.Multiply3x3(Vec3::Up());
		Vec3 capsule_center = in_posA;
		float half_height = capsule->GetCylinderHalfHeight();

		Vec3 A = capsule_center - axis * half_height;
		Vec3 B = capsule_center + axis * half_height;

		float capsule_r = capsule->GetRadius();
		Vec3 C = in_posB;
		float sphere_r = sphere->GetRadius();

		Vec3 AB = B - A;
		float len2 = AB.LengthSq();

		float t = ((C - A).Dot(AB)) / len2;
		t = VxClamp01(t);

		Vec3 pt_on_seg = A + AB * t;
		Vec3 d_seg_sphere = C - pt_on_seg;
		float dist_sq = d_seg_sphere.LengthSq();
		float sum_radius = capsule_r + sphere_r;

		if (dist_sq > sum_radius * sum_radius) return false;

		const float dist = VxSqrt(dist_sq);
		Vec3 nor = d_seg_sphere / dist;
		//sum_radius - dist, //dummy penetration remove later

		o_manifold.normal = nor;
		o_manifold.numManifoldPoints = 1;
		o_manifold.points[0] = {
			pt_on_seg + nor * capsule_r, //point on capsule
			sum_radius - dist, //dummy penetration remove later
			C - nor * sphere_r, //point on sphere
		};
		return true;
	}



	bool CapsuleVsCapsule(const Shape* in_capsule_a, const Vec3& in_posA, const Quat& in_orientationA, const Shape* in_capsule_b, const Vec3& in_posB, const Quat& in_orientationB, ContactManifold& o_manifold)
	{
		const CapsuleShape* capsule_a = nullptr;
		const CapsuleShape* capsule_b = nullptr;

		std::tie(capsule_a, capsule_b) = GetShapesType<CapsuleShape, CapsuleShape>(in_capsule_a, in_capsule_b);
		VX_ASSERT_WARN_RETURN(capsule_a && capsule_b, false, "either both capsule does not exist, cast failed");

		Vec3 pA;
		Vec3 pB;

		Vec3 axisA;
		Vec3 axisB;

		{
			Vec3 A0;
			Vec3 A1;

			Vec3 B0;
			Vec3 B1;

			{
				Mat44 transformA = Mat44::RotationTranslation(in_orientationA, in_posA);
				Mat44 transformB = Mat44::RotationTranslation(in_orientationB, in_posB);

				axisA = transformA.GetAxisY();
				axisB = transformB.GetAxisY();

				float ehA = capsule_a->GetCylinderHalfHeight();
				float ehB = capsule_b->GetCylinderHalfHeight();

				A0 = transformA.GetTranslation() - axisA * ehA;
				A1 = transformA.GetTranslation() + axisA * ehA;

				B0 = transformB.GetTranslation() - axisB * ehB;
				B1 = transformB.GetTranslation() + axisB * ehB;
			}

			Geometry::ClosestPtSegmentSegment(A0, A1, B0, B1, pA, pB);
		}

		Vec3 d_ab = pB - pA;
		float ra = capsule_a->GetRadius();
		float rb = capsule_b->GetRadius();
		float sum_radius = ra + rb;
		float dist_sq = d_ab.LengthSq();

		if (dist_sq > sum_radius * sum_radius) return false;

		float dist = VxSqrt(dist_sq);
		Vec3 n;
		if (dist_sq > 1e-6f)
			n = d_ab / dist;
		else
		{
			n = axisA.Cross(axisB);
			float dist2 = n.LengthSq();
			n = (dist2 > 1e-6f) ? n / VxSqrt(dist2) : Vec3(0.0f, 1.0f, 0.0f);
		}


		o_manifold.normal = n;
		o_manifold.numManifoldPoints = 1;
		o_manifold.points[0] = {
			pA + n * ra,
			sum_radius - dist, //dummy
			pB - n * rb
		};
		return true;
	}

	bool CapsuleVsPlane(const Shape* in_capsule, const Vec3& in_posA, const Quat& in_orientationA, const Shape* in_sphere, const Vec3& in_posB, const Quat& in_orientationB, ContactManifold& o_manifold)
	{
		const CapsuleShape* capsule = nullptr;
		const PlaneShape* plane = nullptr;

		std::tie(capsule, plane) = GetShapesType<CapsuleShape, PlaneShape>(in_capsule, in_sphere);
		VX_ASSERT_WARN_RETURN(capsule && plane, false, "either capsule plane does not exist, cast failed");

		Vec3 plane_n = plane->GetNormal();
		float plane_offset = plane->GetOffset();

		Mat44 capsule_transform = Mat44::RotationTranslation(in_orientationA, in_posA);

		Vec3 center = capsule_transform.GetTranslation();
		Vec3 axis = capsule_transform.GetAxisY();

		float h = capsule->GetCylinderHalfHeight();
		float r = capsule->GetRadius();

		const Mat44 plane_transform = Mat44::RotationTranslation(in_orientationB, in_posB);

		const Vec3 a = center + axis * h;
		const Vec3 b = center - axis * h;


		const Vec3 local_a = plane_transform.TransformInverse(a);
		const Vec3 local_b = plane_transform.TransformInverse(b);
		float da = plane_n.Dot(local_a) - plane_offset;
		float db = plane_n.Dot(local_b) - plane_offset;

		/// capsule as ref A -> B (capsule -> plane)
		Vec3 nor = plane_transform.Multiply3x3(-plane_n).Normalised();
		o_manifold.normal = nor;
		auto& manifold_pts = o_manifold.points;
		int pts_found = 0;

		if (da <= r)
		{
			Vec3 local_pt_on_plane = local_a - plane_n * da;
			manifold_pts[pts_found++] =
			{
				a + nor * r, //pt on capsule
				r - da, //dummy
				plane_transform.Transform(local_pt_on_plane) //pt on plane
			};
		}
		if (db <= r)
		{
			Vec3 local_pt_on_plane = local_b - plane_n * db;
			manifold_pts[pts_found++] =
			{
				b + nor * r, //pt on capsule
				r - db, //dummy
				plane_transform.Transform(local_pt_on_plane) //pt on plane
			};
		}
		o_manifold.numManifoldPoints = pts_found;


		/// TO-DO: this is just a hack; this is a serious bug 
		/// we should have an overlap and generate no points
		///		 
		if (pts_found <= 0) return false;
		VX_ASSERT_WARN(pts_found != 0, "About to generate a manifold with zero points");

		SortContactManifold_Deepest(manifold_pts.data(), pts_found);
		return pts_found;
	}



	bool BoxVsSphere(const Shape* in_box, const Vec3& in_posA, const Quat& in_orientationA, const Shape* in_sphere, const Vec3& in_posB, const Quat& in_orientationB, ContactManifold& o_manifold)
	{
		const BoxShape* box = static_cast<const BoxShape*>(in_box);
		const SphereShape* sphere = static_cast<const SphereShape*>(in_sphere);
		VX_ASSERT_WARN_RETURN(box && sphere, false, "either box or sphere does not exist, cast failed");


		//sphere center to box coordinates
		const Vec3 sphere_center = in_posB;

		const Mat44 box_trans = Mat44::RotationTranslation(in_orientationA, in_posA);
		const Vec3 rel_center = box_trans.TransformInverse(sphere_center);

		const float radius = sphere->GetRadius();
		const Vec3 half_extents = box->GetHalfExtents();

		//if (VxAbs(rel_center.X()) - radius > half_extents.X() ||
		//	VxAbs(rel_center.Y()) - radius > half_extents.Y() ||
		//	VxAbs(rel_center.Z()) - radius > half_extents.Z())
		//	return false;

		if (Vec3::GreaterAny(rel_center.Abs() - Vec3(radius), half_extents))
			return false;

		Vec3 closest_pt = Vec3::Clamp(rel_center, -half_extents, half_extents);

		//float dist_sq = (closest_pt - rel_center).LengthSq();
		float dist_sq = (rel_center - closest_pt).LengthSq();
		if (dist_sq > radius * radius) return false;


		Vec3 closest_pt_world = box_trans.Transform(closest_pt);
		Vec3 n;
		float penetration;// = radius - VxSqrt(dist_sq);

		/// Rare case pick closest face
		/// this breaks when box consumes/sphere center in in box volume 
		/// cause the contact normal to be zero 
		/// meaning this manifold is invalid as it wouldnt resolve 
		/// 
		/// quick solve 
		/// n = closest worlfd - sphere 
		/// if n == 0
		/// 
		/// 
		/// actual
		/// dist_sq == 0
		/// know handle box as point 
		/// at the moment since its the box local space 
		/// normalise sphere rel_center as -direction 
		/// 
		/// and now couple of options 
		/// 1 find closest surface on box 
		/// 2 find closest vertex on box 
		/// 
		/// the closest - rel_center length equals the peneration
		/// 

		/// Box -> Sphere 
		//if (!VxApprox(dist_sq, 0.0f))
		if (dist_sq > 1e-6f)
		{
			/// box -> sphere 
			/// d_bs = sphere_position - box_position
			/// 
			n = (sphere_center - closest_pt_world).Normalised();

			//alternative in world space
			Vec3 nor = (rel_center - closest_pt).Normalised();
			n = box_trans.Multiply3x3(nor);
			penetration = radius - VxSqrt(dist_sq);
		}
		else
		{
			//normal towards box
			Vec3 dist_to_face = half_extents - rel_center.Abs();

			int axis = static_cast<int>(dist_to_face.MinAxis());

			Vec3 nor = Vec3::Zero();
			nor[axis] = (rel_center[axis] >= 0.0f) ? 1.0f : -1.0f;

			closest_pt = rel_center;
			closest_pt[axis] = nor[axis] * half_extents[axis];
			closest_pt_world = box_trans.Transform(closest_pt);

			//world normal 
			n = box_trans.Multiply3x3(nor);

			penetration = radius + dist_to_face[axis];
		}


		o_manifold.normal = n;
		o_manifold.numManifoldPoints = 1;
		o_manifold.points[0] =
		{
			closest_pt_world, //on box
			penetration, //dummy
			closest_pt_world - n * penetration //on sphere surface 
		};


		return true;
	}




	bool BoxVsPlane(const Shape* in_box, const Vec3& in_posA, const Quat& in_orientationA, const Shape* in_plane, const Vec3& in_posB, const Quat& in_orientationB, ContactManifold& o_manifold)
	{
		const PlaneShape* plane = static_cast<const PlaneShape*>(in_plane);
		const BoxShape* box = static_cast<const BoxShape*>(in_box);
		VX_ASSERT_WARN_RETURN(plane && box, false, "either plane or box does not exist, cast failed");

		const Mat44 plane_transform = Mat44::RotationTranslation(in_orientationB, in_posB);

		const Vec3 plane_n = plane->GetNormal().Normalised();
		const Vec3& world_nor = plane_transform.Multiply3x3(plane_n).Normalise();

		/// d is the constant offset the plane in its space
		/// of the plane has transform and been transformed 
		/// then consider that tranlation and orignal 
		/// offset
		const float d = plane->GetOffset() + world_nor.Dot(plane_transform.GetTranslation());

		//box is considered to be axis aligned 
		std::array<Vec3, 4> face_vert;
		{
			Mat44 box_T = Mat44::RotationTranslation(in_orientationA, in_posA);
			face_vert = box->GetSupportingFaceVerticesOBB(box_T, -world_nor);
		}


		Vec3 clipped[8];
		int count = ClipPolygonAgainstPlane(face_vert.data(), 4, world_nor, d, clipped);

		if (count <= 0) return false;

		ManifoldPoint manifold_pts[8];
		/// box as ref A->B(box->plane) -plane_nor
		o_manifold.normal = -world_nor;


		int pts_found = 0;
		for (int i = 0; i < count; i++)
		{
			//float dist = plane_n.Dot(local_sphere) - plane_shape->GetOffset();
			//float depth = d - clipped[i].Dot(plane_n);
			//float depth = clipped[i].Dot(plane_n) - d;
			float depth = d - world_nor.Dot(clipped[i]);

			if (depth < 1e-6f) continue;
			VX_ASSERT_WARN(depth > 1e-6f, "The penetration need to be positive to be a positive intersection");

			ManifoldPoint cp;
			cp.pointA = clipped[i] + world_nor * depth;
			cp.peneration = depth;
			cp.pointB = clipped[i];
			//Vec3 local_pt_on_plane = local_b - plane_n * db;

			manifold_pts[pts_found++] = cp;
		}




		/// TO-DO: this is just a hack; this is a serious bug 
		/// we should have an overlap and generate no points
		///		 
		if (pts_found <= 0) return false;
		VX_ASSERT_WARN(pts_found != 0, "About to generate a manifold witih zero points");

		if (pts_found <= 4)
		{
			//Geometry::SortContactManifold_Deepest(manifold_pts, pts_found);
			o_manifold.Store(manifold_pts, pts_found);
			return true;
		}

		auto& pts = o_manifold.points;
		ReduceContactManifoldTo4_Geometric(manifold_pts, pts_found, &o_manifold.points[0]);
		o_manifold.numManifoldPoints = 4;

		return true;
	}





	bool BoxVsBox(const Shape* in_box_a, const Vec3& in_posA, const Quat& in_orientationA, const Shape* in_box_b, const Vec3& in_posB, const Quat& in_orientationB, ContactManifold& o_manifold)
	{
		const BoxShape* box_a = static_cast<const BoxShape*>(in_box_a);
		const BoxShape* box_b = static_cast<const BoxShape*>(in_box_b);
		VX_ASSERT_WARN_RETURN(box_a && box_b, false, "either both boxes does not exist, cast failed");


		float min_overlap = FLT_MAX; //minimum translation vector
		int best_axis_type = -1; //0= A face, 1= B face, 2 = edge-edge
		int best_a = -1, best_b = -1;

		Vec3 perp_vector = Vec3(0.0f);

		enum class InBoxes : uint8 { A = 0, B = 1 };
		const Mat44 boxes_T[2] = {
			Mat44::RotationTranslation(in_orientationA, in_posA),
			Mat44::RotationTranslation(in_orientationB, in_posB)
		};

		{
#pragma region SAT
			const Mat44& box_aT = boxes_T[static_cast<int>(InBoxes::A)];
			const Mat44& box_bT = boxes_T[static_cast<int>(InBoxes::B)];

			const Vec3 box_aR[3] = { box_aT.GetColumn(0), box_aT.GetColumn(1), box_aT.GetColumn(2) };
			const Vec3 box_bR[3] = { box_bT.GetColumn(0), box_bT.GetColumn(1),box_bT.GetColumn(2) };

			const float k_edge_bias = 0.98f; //5% penalty for edges
			const float k_edge_slop = 0.1f; //1cm linear penalty
			float epsilon = 1e-2f;
			// Compute common subexpressions. Add in an epsilon term to
			// counteract arithmetic errors when two edges are parallel and
			// their cross product is (near) null (see text for details)
			float R[3][3], AbsR[3][3];
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j)
				{
					R[i][j] = box_aR[i].Dot(box_bR[j]);
					AbsR[i][j] = VxAbs(R[i][j]) + epsilon;
				}



			// Compute rotation matrix expressing b in a’s coordinate frame
			// Compute translation vector t
			Vec3 d_ab = box_bT.GetTranslation() - box_aT.GetTranslation();
			// Bring translation into a’s coordinate frame
			Vec3 box_b_relA = Vec3(d_ab.Dot(box_aR[0]), d_ab.Dot(box_aR[1]), d_ab.Dot(box_aR[2]));

			Vec3 boxA_half = box_a->GetHalfExtents();
			Vec3 boxB_half = box_b->GetHalfExtents();

			Vec3 best_axis;
			float ra, rb;

			// Test axes L = A0, L = A1, L = A2
			for (int i = 0; i < 3; i++)
			{
				ra = boxA_half[i];
				rb = boxB_half[0] * AbsR[i][0] +
					boxB_half[1] * AbsR[i][1] +
					boxB_half[2] * AbsR[i][2];

				float proj = box_b_relA[i];
				float overlap = ra + rb - VxAbs(proj);
				if (overlap < -kEpsilon) return false;

				if (overlap < min_overlap)
				{
					min_overlap = overlap;
					//best_axis = local_axes0[i];///THIS THIS
					best_axis = box_aR[i] * (proj < 0 ? -1.0f : 1.0f); //A's face normal ///THIS THIS
					best_axis_type = 0;
					best_a = i;
				}
			}

			// Test axes L = B0, L = B1, L = B2
			for (int i = 0; i < 3; i++)
			{
				ra = boxA_half[0] * AbsR[0][i] +
					boxA_half[1] * AbsR[1][i] +
					boxA_half[2] * AbsR[2][i];
				rb = boxB_half[i];


				float proj = d_ab.Dot(box_bR[i]);
				//float overlap = ra + rb - VxAbs(proj);
				float overlap = ra + rb - VxAbs(proj);
				if (overlap < -kEpsilon) return false;

				///if close to box a's overlap 
				/// ensure to priories A overlap to prevent flickering
				//if (overlap < (min_overlap*0.95)-0.01f)
				if (overlap < min_overlap)
				{
					min_overlap = overlap;
					//best_axis = local_axes1[i];///THIS THIS
					best_axis = box_bR[i] * (proj > 0 ? -1.0f : 1.0f); //A's face normal ///THIS THIS
					best_axis_type = 1; //b face
					best_b = i;
				}
			}


			///Edge - edge
			const float k_edge_eps_sq = 5e-4f;//  0.05f;// 1e-3f; //extra penalty for almost parallel
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					Vec3 axis = box_aR[i].Cross(box_bR[j]);

					float len_sq = axis.LengthSq();
					if (len_sq < k_edge_eps_sq) continue; //nearly parellel 
					axis /= VxSqrt(len_sq);

					float ra = boxA_half[(i + 1) % 3] * AbsR[(i + 2) % 3][j] +
						boxA_half[(i + 2) % 3] * AbsR[(i + 1) % 3][j];
					float rb = boxB_half[(j + 1) % 3] * AbsR[i][(j + 2) % 3] +
						boxB_half[(j + 2) % 3] * AbsR[i][(j + 1) % 3];

					//errixson formula t project dist of t oonto axis on A's frame
					float proj = (
						box_b_relA[(i + 2) % 3] * R[(i + 1) % 3][j] -
						box_b_relA[(i + 1) % 3] * R[(i + 2) % 3][j]);

					float overlap = ra + rb - VxAbs(proj);
					if (overlap < -kEpsilon) return false;

					/// hack bias 
					/// a give hard penalty to edge to edge to favour face contact
					//if (overlap < min_overlap)
					if (overlap < (min_overlap * k_edge_bias) - k_edge_slop)
					{
						min_overlap = overlap;
						//best_axis = axis * ((proj < 0.0f) ? 1.0f : -1.0f); /// TEST TEST
						best_axis = axis * ((proj > 0.0f) ? 1.0f : -1.0f); /// TEST TEST
						best_axis_type = 2;
						best_a = i;
						best_b = j;
					}
				}
			}

#pragma endregion
			perp_vector = best_axis;
		}

#if VX_DEBUG_CONTACT_GENERATION
		BoxVsBoxSAT debug_info;
		perp_vector.Store(debug_info.best_axis);
		debug_info.best_axis_type = best_axis_type;
		debug_info.axisMinOverlap = min_overlap;
		//hypotectic poiny 
		{
			Vec3 pt = (in_posA + in_posB) * 0.5f;
			pt.Store(debug_info.manifoldCenter);
		}
#endif // VX_DEBUG_CONTACT_GENERATION

		/// face - face, face - vertex
		if (best_axis_type == 0 || best_axis_type == 1)
		{
			const BoxShape* boxes[2] = { box_a, box_b };
			int best_axes[2] = { best_a, best_b };
			
			int ref = best_axis_type;
			int inc = 1 - ref;

			const BoxShape* ref_box = boxes[ref];
			const BoxShape* inc_box = boxes[inc];
			int ref_axis_idx = best_axes[ref];

			/// to flag the manifold as body used as ref as change 
			/// so the dispatcher could handle the swap
			bool reorientate_manifold = (ref != 0);

			Mat44 ref_transform = boxes_T[ref];
			Mat44 inc_transform = boxes_T[inc];


			//ensure n point from ref -> inc
			Vec3 ref_center = ref_transform.GetTranslation();

			//ref face nor in world space
			Vec3 face_n = perp_vector;

			int axis_0 = (ref_axis_idx + 1) % 3;
			Vec3 ref_tangent = ref_transform.GetColumn(axis_0).Normalised();
			Vec3 ref_bitangent = face_n.Cross(ref_tangent).Normalised();
			ref_tangent = ref_bitangent.Cross(face_n).Normalised();

			float ref_tangent_half = ref_box->GetHalfExtents()[axis_0];
			float ref_bitangent_half = ref_box->GetHalfExtents()[(ref_axis_idx + 2) % 3];



			float ref_half = ref_box->GetHalfExtents()[ref_axis_idx];
			Vec3 ref_face_point = ref_center + face_n * ref_half;

			/// i think this face vert is the face on the inc box
			const auto& inc_face_vert = inc_box->GetSupportingFaceVerticesOBB(inc_transform, -face_n);

			std::array<Vec3, 8> clipped_vert_pts;
			int clipped_verts = ClipFaceToBoxSidePlanes(
				inc_face_vert.data(), inc_face_vert.size(),
				ref_face_point, ref_tangent, ref_tangent_half,
				ref_bitangent, ref_bitangent_half, clipped_vert_pts.data());

			o_manifold.normal = (!reorientate_manifold) ? face_n : -face_n;
			ManifoldPoint manifold_pts[8];
			int pts_found = 0;

			///offset of face from it center could essential be dir/axis*half_extent.Dot(face_n)
			float d = ref_face_point.Dot(face_n);
			constexpr float k_depth_tolenrance = 1e-3f;
			//build manifold
			for (int i = 0; i < clipped_verts; i++)
			{
				//actually the verts point are points on incident box 
				Vec3 vert_pt = clipped_vert_pts[i];
				////float plane_depth = d - face_n.Dot(vert_pt); ///THIS THIS
				float plane_depth = d - vert_pt.Dot(face_n); ///THIS THIS
				if (plane_depth < k_depth_tolenrance)continue;

				auto& manifold_pt = manifold_pts[pts_found++];
				manifold_pt.pointB = vert_pt + face_n * plane_depth; //point on self 
				manifold_pt.peneration = plane_depth; //plane depth in most case should be negative, but to be safe Use Abs.//VxMin(-plane_depth, min_overlap);
				manifold_pt.pointA = vert_pt;// plane clip points on the inclident box  


#if VX_DEBUG_CONTACT_GENERATION
				(i < debug_info.pointsGenerated.size() - 3) ? vert_pt.Store(debug_info.pointsGenerated[debug_info.generatedPt++]) : void();
#endif // VX_DEBUG_CONTACT_GENERATION
			}

			(reorientate_manifold) ? SwapContactManifoldPoints(manifold_pts, pts_found) : void(0);




			/// TO-DO: this is just a hack; this is a serious bug 
			/// we should have an overlap and generate no points
			///		 
			if (pts_found <= 0) return false;
			VX_ASSERT_WARN(pts_found != 0, "Manifold with zero points is about to be generated");

			if (pts_found <= 4)
			{
				//	Geometry::SortContactManifold_Deepest(manifold_pts, pts_found);
				o_manifold.Store(manifold_pts, pts_found);
			}
			else
			{
				ReduceContactManifoldTo4_Geometric(manifold_pts, pts_found, o_manifold.points.data());
				o_manifold.numManifoldPoints = 4;
			}

#if VX_DEBUG_CONTACT_GENERATION
			debug_info.hasPlane = true;
			debug_info.planeSize = { ref_tangent_half, ref_bitangent_half };
			ref_tangent.Store(debug_info.planeTangent);
			ref_bitangent.Store(debug_info.planeBiTangent);
			o_manifold.normal.Store(debug_info.contactNormal);
			ref_face_point.Store(debug_info.manifoldCenter);//update hypotectic to correct pot
			debug_info.ref = (best_axis_type == 0) ? (BoxShape*)(in_box_a) : (BoxShape*)(in_box_b);
			debug_info.inc = (best_axis_type == 0) ? (BoxShape*)(in_box_b) : (BoxShape*)(in_box_a);
			if (clipped_verts < 3) //probable vertex to face contacrt
				debug_info.best_axis_type = 3;


			///Actual contact points
			for (int i = 0; i < o_manifold.numManifoldPoints; ++i)
				o_manifold.points[i].pointA.Store(debug_info.contactPoints[debug_info.numContactPt++]);
			///Genereted points
			debug_info.generatedPt = VxMin(debug_info.generatedPt, pts_found);
			sBoxVsBoxSATDebugInstances.push_back(debug_info);
#endif // VX_DEBUG_CONTACT_GENERATION

			return true;
		}
		else if (best_axis_type == 2)
		{
			Vec3 half_a = box_a->GetHalfExtents();
			Vec3 half_b = box_b->GetHalfExtents();

			//normalise 
			Vec3 n = perp_vector;
			Vec3 edge_dirA = boxes_T[static_cast<int>(InBoxes::A)].GetColumn(best_a).Normalised();
			Vec3 edge_dirB = boxes_T[static_cast<int>(InBoxes::B)].GetColumn(best_b).Normalised();


			Vec3 pa = box_a->GetEdgeCenter(boxes_T[static_cast<int>(InBoxes::A)], best_a, n);
			Vec3 pb = box_b->GetEdgeCenter(boxes_T[static_cast<int>(InBoxes::B)], best_b, -n);

			Vec3 edge_a0 = pa - edge_dirA * half_a[best_a];
			Vec3 edge_a1 = pa + edge_dirA * half_a[best_a];

			Vec3 edge_b0 = pb - edge_dirB * half_b[best_b];
			Vec3 edge_b1 = pb + edge_dirB * half_b[best_b];


			Vec3 closest0;
			Vec3 closest1;
			Geometry::ClosestPtSegmentSegment(edge_a0, edge_a1, edge_b0, edge_b1, closest0, closest1);


			o_manifold.normal = n;
			auto& manifold_pt = o_manifold.points[o_manifold.numManifoldPoints++];

			manifold_pt.pointA = closest0;
			//cp.peneration = (closest1 - closest0).Dot(n);// min_overlap;
			manifold_pt.peneration = min_overlap;
			manifold_pt.pointB = closest1;


#if VX_DEBUG_CONTACT_GENERATION
			debug_info.ref = (BoxShape*)in_box_a;
			debug_info.inc = (BoxShape*)in_box_b;
			debug_info.hasPlane = true;
			debug_info.planeSize = { half_a[best_a], half_b[best_b] };
			VX_ASSERT(edge_dirA.IsNormalised(1e-6f), "edge dir needs to be normalised");
			VX_ASSERT(edge_dirB.IsNormalised(1e-6f), "edge dir needs to be normalised");
			edge_dirA.Store(debug_info.planeTangent);
			edge_dirB.Store(debug_info.planeBiTangent);
			n.Store(debug_info.contactNormal);


			pa.Store(debug_info.edgeCenter0);
			pb.Store(debug_info.edgeCenter1);

			edge_a0.Store(debug_info.edgeA0);
			edge_a1.Store(debug_info.edgeA1);
			edge_b0.Store(debug_info.edgeB0);
			edge_b1.Store(debug_info.edgeB1);
			debug_info.isEdgeDetection = true;

			sBoxVsBoxSATDebugInstances.push_back(debug_info);
#endif // VX_DEBUG_CONTACT_GENERATION

			return true;
		}


		return false; //BoxVsBox()
	}

	bool BoxVsCapsule(const Shape* in_box, const Vec3& in_posA, const Quat& in_orientationA, const Shape* in_capsule, const Vec3& in_posB, const Quat& in_orientationB, ContactManifold& o_manifold)
	{
		const BoxShape* box = nullptr;
		const CapsuleShape* capsule = nullptr;

		std::tie(box, capsule) = GetShapesType<BoxShape, CapsuleShape>(in_box, in_capsule);
		VX_ASSERT_WARN_RETURN(box && capsule, false, "shape(s) null");


#define USE_NEW 1
#if USE_NEW

		const float r = capsule->GetRadius();
		const Vec3 he = box->GetHalfExtents();

		const Vec3 c_ws = in_posB;
		Vec3 axis_ws = in_orientationB.RotateAxisY() * capsule->GetCylinderHalfHeight();

		Vec3 pA_ws = c_ws - axis_ws;
		Vec3 pB_ws = c_ws + axis_ws;

		//Mat44 box_tran = Mat44::RotationTranslation(in_orientationA, in_posA);
		//Vec3 A = box_tran.TransformInverse(pA_ws);
		//Vec3 B = box_tran.TransformInverse(pB_ws);
		Vec3 A = in_orientationA.InverseRotate(pA_ws - in_posA);
		Vec3 B = in_orientationA.InverseRotate(pB_ws - in_posA);

		float t;
		//closeset pt on segment to box origin
		Vec3 seg_closest = Geometry::ClosestPtPointSegment(Vec3(0.0f), A, B, t);

		//closest pt on box to segment
		Vec3 box_closest = Vec3::Clamp(seg_closest, -he, he);

		Vec3 delta = seg_closest - box_closest;
		float dist_sq = delta.LengthSq();

		if (dist_sq < 1e-12f) //inside or very close
		{
			Vec3 penetration = he - seg_closest.Abs();
			int axis = static_cast<int>(penetration.MinAxis());

			Vec3 n = Vec3(0.0f);
			n[axis] = (seg_closest[axis] > 0.0f) ? 1.0f : -1.0f;

			//Vec3 n_ws = box_tran.Multiply3x3(n).Normalised(); //in_orien.Rotate(n)
			Vec3 n_ws = in_orientationA.Rotate(n).Normalised();

			//Vec3 p_box_ws = box_tran.Transform(box_closest);
			Vec3 p_box_ws = in_orientationA.Rotate(box_closest) + in_posA;
			Vec3 p_caps_ws = p_box_ws - n_ws * r;

			//later have a helper function .AddPoint(pA, penetration, pB)
			o_manifold.points[0] = {
				p_box_ws, 
				penetration.MinComponent(),
				p_box_ws - n_ws * penetration
			};

			o_manifold.normal = n_ws;
			o_manifold.numManifoldPoints = 1;
			return true;
		}

		float dist = VxSqrt(dist_sq);
		float penetration = r - dist;

		if (penetration <= 0.0f) return false;

		Vec3 n_ls = delta / dist;
		//Vec3 n_ws = box_tran.Multiply3x3(n_ls).Normalised();
		Vec3 n_ws = in_orientationA.Rotate(n_ls).Normalised();

		//Vec3 p_box_ws = box_tran.Transform(box_closest);
		//Vec3 p_cap_ws = box_tran.Transform(seg_closest) - n_ws * r;
		Vec3 p_box_ws = in_orientationA.Rotate(box_closest) + in_posA;
		Vec3 p_cap_ws = (in_orientationA.Rotate(seg_closest) + in_posA) - n_ws * r;


		int pt_count = 0;
		o_manifold.points[pt_count++] =
		{
			p_box_ws, 
			penetration,
			p_box_ws - n_ws * penetration
		};

		Vec3 closest_on_seg_A = Geometry::ClosestPtPointSegment(A, A, B, t);
		Vec3 closest_on_seg_B = Geometry::ClosestPtPointSegment(B, A, B, t);

		Vec3 candidates[2] = { closest_on_seg_A, closest_on_seg_B };

		for (int i = 0; i < 2; ++i)
		{
			Vec3 c = Vec3::Clamp(candidates[i], -he, he);
			Vec3 d = candidates[i] - c;
			float dist = d.Length();

			float penetration = r - dist;
			if (penetration <= 0.0f)
				continue;

			Vec3 n = (dist > 1e-6f) ? d / dist : n_ls;

			//Vec3 nW = box_tran.Multiply3x3(c).Normalised();
			Vec3 nW = in_orientationA.Rotate(c).Normalised();

			//Vec3 pB = box_tran.Transform(c);
			Vec3 pB = in_orientationA.Rotate(c) + in_posA;;
			Vec3 pA = pB - nW * penetration;

			o_manifold.points[pt_count++] ={
				pB,
				penetration,
				pA
			};

		}

		o_manifold.numManifoldPoints = pt_count;

	
		o_manifold.normal = n_ws;
		o_manifold.numManifoldPoints = pt_count;
		if(pt_count > 1)
			SortContactManifold_Deepest(o_manifold.points.data(), pt_count);
		return true;

		

#else
		///OLD OLD 
				/// 
				/// 
				///// 
				/// Solve the collision in box space
				/// so box become axis aligned.
				/// This avoids transforming all box corners and simple math

		Mat44 box_trans = Mat44::RotationTranslation(in_orientationA, in_posA);
		Mat44 capsule_trans = Mat44::RotationTranslation(in_orientationB, in_posB);

		const Vec3 box_half_extents = box->GetHalfExtents();

		const Vec3& capsule_center = in_posB;
		Vec3 capsule_axis_ws = capsule_trans.GetAxisY() * capsule->GetCylinderHalfHeight();
		Vec3 pA_ws = capsule_center - capsule_axis_ws;
		Vec3 pB_ws = capsule_center + capsule_axis_ws;


		Vec3 A = box_trans.TransformInverse(pA_ws);
		Vec3 B = box_trans.TransformInverse(pB_ws);
		Vec3 seg_dir = B - A;


		float t = 0.0f;
		float len_sq = seg_dir.LengthSq();
		if (len_sq > 1e-6f)
			t = VxClamp01(-A.Dot(seg_dir) / len_sq);
		Vec3 pt_seg = A + seg_dir * t;


		Vec3 pt_box = Vec3::Clamp(pt_seg, -box_half_extents, box_half_extents);

		Vec3 d = pt_seg - pt_box;
		float dist_sq = d.LengthSq();
		float r = capsule->GetRadius();

		if (dist_sq > r * r) return false;


		Vec3 dist_to_edge = box_half_extents - pt_seg.Abs();
		int axis = static_cast<int>(dist_to_edge.MinAxis());


		Vec3 n_ls = Vec3::Zero();
		//n_ls[axis] = (pt_seg[axis] > 0.0f) ? 1.0f : -1.0f;
		if (dist_sq > 1e-7f)
			n_ls = d / VxSqrt(dist_sq);
		else//inside/on surface min axis
		{
			n_ls[axis] = (pt_seg[axis] > 0.0f) ? 1.0f : -1.0f;
		}
		Vec3 n_ws = box_trans.Multiply3x3(n_ls).Normalised();

		o_manifold.normal = n_ws;
		auto& manifold_pt = o_manifold.points;
		int pts_found = 0;

		//if (VxAbs((seg_dir.Normalised()).Dot(n_ls)) < 5e-2f) //check parallelism
		Vec3 seg_n = (len_sq > 1e-6f) ? seg_dir / VxSqrt(len_sq) : Vec3(0, 1, 0);
		if (VxAbs((seg_n).Dot(n_ls)) < 1.0f) //check parallelism
		{
			float t_min = 0.0f, t_max = 1.0f;
			//1D aabb clip (slab clipping)
			for (int i = 0; i < 3; ++i)
			{
				if (i == axis)continue;
				float denom = B[i] - A[i];
				if (VxAbs(denom) > 1e-6f)
				{
					float t0 = (-box_half_extents[i] - A[i]) / denom;
					float t1 = (box_half_extents[i] - A[i]) / denom;
					t_min = VxMax(t_min, VxMin(t0, t1));
					t_max = VxMin(t_max, VxMax(t0, t1));
					//t_max = VxMax(t_max, VxMax(t0, t1));
				}
			}

			if (t_max > t_min)
			{
				Vec3 pts[2] = { A + seg_dir * t_min, A + seg_dir * t_max };
				for (int i = 0; i < 2; ++i)
				{
					Vec3 p_ws = box_trans.Transform(pts[i]);
					Vec3 p_box_ls = Vec3::Clamp(pts[i], -box_half_extents, box_half_extents);
					Vec3 local_d = pts[i] - p_box_ls;

					float dist = local_d.Length();
					float penetration = r - dist;

					//float p_dist = (pts[i] - Vec3::Clamp(pts[i], -box_half_extents, box_half_extents)).Length();
					if (penetration > 0.0f)
					{
						Vec3 curr_n_ls = (dist > 1e-6f) ? local_d / dist : n_ls;
						Vec3 curr_n_ws = box_trans.Multiply3x3(curr_n_ls).Normalised();
						manifold_pt[pts_found++] = {
							box_trans.Transform(p_box_ls), // pt on box
							penetration,
							p_ws - curr_n_ws * r, //pt on capsule
						};
					}
				}

				if (pts_found > 0)
				{
					o_manifold.normal = n_ws;
					o_manifold.numManifoldPoints = pts_found;
					SortContactManifold_Deepest(o_manifold.points.data(), pts_found);
					return true;
				}
			}
		}


		//fallback 
		float pentration = r - VxSqrt(dist_sq);
		manifold_pt[pts_found++] = {
			box_trans.Transform(pt_box), //on box
			pentration,
			box_trans.Transform(pt_seg) - n_ws * r, // on capsule
		};
		o_manifold.numManifoldPoints = pts_found;
		return true;
#endif // USE_NEW
	}

} //namespace vx::Narrowphase