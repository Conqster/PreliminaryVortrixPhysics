#pragma once 

#include "Vortrix/Geometry/GeometricAlgorithms.h"
#include "WorldQuery.h"
#include "Vortrix/Dynamics/Body/Body.h"

#include "RayCast.h"

#include "Vortrix/Visuals/Renderers.h"
//#include "Core/Colours.h"

namespace vx {

	using RayShapeFn = bool(*)(const struct RayCast&, const class Shape*, struct RaycastHit&);


	template<typename Cast, typename CasterDispatch>
	struct WorldQueryContext
	{
		//const RayCast& worldRay;
		const Cast& worldCast;
		class QueryProcessor& queryProcessor;
		const CasterDispatch* dispatchTable;
		DebugGizmosRenderer* debugRenderer = nullptr;

		//quick test 
		float _t_min = kMaxf;

		bool VisitBody(Body& body)
		{
			RaycastHit hit_result;
			Vec3 t = body.GetPosition();
			Quat q = body.GetOrientation();
			EShapeType shape_type = body.GetShape()->GetType();

			////Cast& local_cast = worldCast.Transformed() // as it could be custom
			//Mat44 body_trans = Mat44::RotationTranslation(q, t);
			//Vec3 ray_origin = body_trans.MultiplyAffine(worldCast.origin);
			//Vec3 ray_dir = body_trans.MultiplyAffine(worldCast.origin + worldCast.direction) - ray_origin;

			//Vec3 disp = ray_origin + ray_dir * worldCast.length;
			//Cast& local_cast = Cast(ray_origin, disp);

			////AABB aabb = body.GetShape()->GetLocalBounds();
			////AABB aabb = body.GetAABBWorld();
			////Body body 
			//AABB aabb = body.ComputeAABBWorld();
			//Vec3 b_min = aabb.mMin;
			//Vec3 b_max = aabb.mMax;

			//float t_min = -1;

			//RaycastHit old_hit = (HasHit()) ? queryProcessor.Result().hits[0] : RaycastHit();

			//Mat44 inv_tran = Mat44::InverseRotationTranslation(q, t);
			//Vec3 local_origin = inv_tran.MultiplyAffine(worldCast.origin);
			//Vec3 local_disp = inv_tran.Multiply3x3(worldCast.displacement);
			//Vec3 local_inv_dir = Vec3(1.0f) / local_disp;

			//local_origin = worldCast.origin;
			//local_inv_dir = worldCast.invDisplacement;
			//bool hit = Geometry::RayAABB(local_origin, local_inv_dir, b_min, b_max, t_min, old_hit.fraction);
			//Vec3 _intersect;
			////bool hit = Geometry::RayAABB(local_origin, local_inv_dir, b_min, b_max, t_min, _intersect);
			//if (hit && t_min >= 0)
			//{
			//	//if(t_min < old_hit.fraction)
			//	{
			//		hit_result.body = body.GetID();
			//		hit_result.fraction = t_min;
			//		hit_result._min = b_min;
			//		hit_result._max = b_max;
			//		//hit_result._intersect = _intersect;
			//		queryProcessor.AddHit(hit_result);
			//	}
			//}

			Mat44 world_local = Mat44::InverseRotationTranslation(q, t);
			Vec3 local_origin = world_local.MultiplyAffine(worldCast.origin);
			Vec3 local_disp = world_local.Multiply3x3(worldCast.displacement);
			Cast local_cast(local_origin, local_disp);
			//narrowphase
			if (dispatchTable[int(shape_type)](local_cast, body.GetShape(), hit_result))
			{
				//world space normal 
				hit_result.body = body.GetID();

				VX_ASSERT_WARN(hit_result.body.IsValid(), "Invalid Bodiy");
				hit_result.normal = q.Rotate(hit_result.normal);
				queryProcessor.AddHit(hit_result);

				//hit_result.fraction = _t_min;
				return true;
			}
			return false;
		}

		//for now process one node at a time
		//bool VisitNode(const Vec3* bounds_mins, const Vec3* bounds_maxs, int counts)
		bool VisitNode(const Vec3& bounds_min, const Vec3& bounds_max)
		{
			float t_min = 0.0f;

			float t_limit = queryProcessor.EarlyOutFraction();
			//return true;
			bool hit = Geometry::RayAABB(worldCast.origin, worldCast.invDisplacement,
				bounds_min, bounds_max, t_min, t_limit);


			if (debugRenderer)
			{
				Colour col = (!hit) ? Colour::sYellow : Colour::sRed;

				debugRenderer->DrawAABB(bounds_min,
					bounds_max, col);
			}

			return hit;
		}

		//float 3 has hack 
		Float3 VisitNode2(const Vec4& boundsX, 
			const Vec4& boundsY, const Vec4& boundsZ)
		{

			Vec4 result = Geometry::RayAABB_2(worldCast.origin, worldCast.invDisplacement,
				boundsX, boundsY, boundsZ);

			float t_limit = queryProcessor.EarlyOutFraction();

			float tA = (result.X() >= kMaxf || result.X() > t_limit) ? kMaxf : result.X();
			float tB = (result.Y() >= kMaxf || result.Y() > t_limit) ? kMaxf : result.Y();
			
			if (debugRenderer)
			{
				Colour col = (tA == kMaxf) ? Colour::sYellow : Colour::sRed;

				//Vec3 b_min0;
				//Vec3 b_min1;
				//Vec3 b_max0;
				//Vec3 b_max1;
				Vec3 b_min_max01[4];
				for (int i = 0; i < 4; ++i)
					b_min_max01[i] = Vec3(boundsX[i], boundsY[i], boundsZ[i]);
				
				debugRenderer->DrawAABB(b_min_max01[0], b_min_max01[2], col);
				col = (tB == kMaxf) ? Colour::sYellow : Colour::sRed;
				debugRenderer->DrawAABB(b_min_max01[1], b_min_max01[3], col);
			}

			return Float3(tA, tB, 0.0f);
		}

		bool Complete() const { return queryProcessor.ShouldEarlyOut(); }


		//bool OnBroadphaseHit(struct BroadphaseProxy proxy)
		//{

		//}

		bool HasHit()
		{
			return queryProcessor.HasHit();
		}

		void AddHit()
		{
			//queryProcessor.AddHit()
		}
	};

}