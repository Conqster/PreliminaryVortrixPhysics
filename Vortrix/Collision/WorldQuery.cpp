#include "WorldQuery.h"
#include "WorldQueryContext.h"

#include "Shapes/Shape.h"
#include "Shapes/SphereShape.h"
#include "Shapes/CapsuleShape.h"
#include "Shapes/BoxShape.h"
#include "Shapes/PlaneShape.h"
#include "Broadphase/Broadphase.h"

#include "RayCast.h"

namespace vx {

	//struct RayShapeDispatch
	//{
	//	RayShapeFn mDispatchTable[int(EShapeType::Count)];
	//};


	static bool UnsupportedPair(const RayCast&, const Shape* shape, RaycastHit&)
	{
		VX_LOG_WARN("Unsupportd Ray-vs-{", shape->GetShapeTypeName(), "} Dispatch!!!");
		return false;
	}


	VX_INLINE bool RaySphere(const RayCast& local_cast, const Shape* shape, RaycastHit& hit)
	{
		const SphereShape* sphere = static_cast<const SphereShape*>(shape);
		float radius = sphere->GetRadius();

		const Vec3& d = local_cast.displacement;
		const Vec3& origin = local_cast.origin;

		float a = d.Dot(d);
		float b = 2.0f * origin.Dot(d);
		float c = origin.Dot(origin) - (radius * radius);

		//discriminant
		float disc = b * b - 4.0f * a * c;
		if (disc < 0.0f)return false;

		float sqrt_d = VxSqrt(disc);
		float t = (-b - sqrt_d) / (2.0f * a);

		if (t < 0.0f)
			t = (-b + sqrt_d) / (2.0f * a);
		if (t < 0.0f || t>1.0f)return false;
		hit.fraction = t;
		hit.normal = (origin + d * t) / radius;
		return true;
	}

	VX_INLINE bool RayPlane(const RayCast& local_cast, const Shape* shape, RaycastHit& hit)
	{
		const PlaneShape* plane = static_cast<const PlaneShape*>(shape);

		float d = plane->GetOffset();
		Vec3 n = plane->GetNormal();

		Vec3 origin = local_cast.origin;
		Vec3 disp = local_cast.displacement;

		//float t = (d - n.Dot(origin) / n.Dot(disp));
		float t = (d - n.Dot(origin) * n.Dot(local_cast.invDisplacement));


		if (t < 0.0f || t>1.0f)return false;

		hit.fraction = t;
		hit.normal = n;
		return true;
	}



	VX_INLINE bool RayCapsule(const RayCast& local_cast, const Shape* shape, RaycastHit& hit)
	{
		
		const CapsuleShape* capsule = static_cast<const CapsuleShape*>(shape);

		float height = capsule->GetCylinderHalfHeight();// +capsule->GetRadius();
		float radius = capsule->GetRadius();

		const Vec3& d = local_cast.displacement;
		const Vec3& origin = local_cast.origin;

		float hit_y = origin.Y();

		//Vec4 dxdotdz_orix_dot_orix = Vec4::Dot

		///later for future if latter is arbutriy 
		/// Vec3 up_axis_mask = capsule.UpAxis();
		/// d *= (up_axis_mask - Vec3(1.0f)) * Vec3(-1.0f))
		/// float = Vec4::Dot(d, d)
		
		float a = Vec4::Dot(Vec4(d.X(), 0.0f, d.Z(), 0.0f), Vec4(d.X(), 0.0f, d.Z(), 0.0f));
		if (a < 1e-8f) goto hemisphere_test; //parallel

		float b = 2.0f * Vec4::Dot(Vec4(origin.X(), 0.0f, origin.Z(), 0.0f), Vec4(d.X(), 0.0f, d.Z(), 0.0f));
		float c = Vec4::Dot(Vec4(origin.X(), 0.0f, origin.Z(), 0.0f), Vec4(origin.X(), 0.0f, origin.Z(), 0.0f)) - (radius * radius);

		//discriminant
		float discr = b * b - 4.0f * a * c;
		if (discr < 0.0f) goto hemisphere_test; //missed cylinder,


		float sqrt_d = VxSqrt(discr);
		float t = (-b - sqrt_d) / (2.0f * a);

		if (t < 0.0f)
			t = (-b + sqrt_d) / (2.0f * a);
		if (t < 0.0f || t>1.0f)return false;



		hit_y = origin.Y() + d.Y() * t;
		if (VxAbs(hit_y) <= height)
		{
			Vec3 local_hit = (origin + d * t) / radius;
			hit.fraction = t;
			hit.normal = Vec3(local_hit.X(), 0.0f, local_hit.Z());
			return true;
		}

		//////////////////////////////////////
		// HEMISPHERE TEST
		//////////////////////////////////////
		hemisphere_test:
		float along = d.Dot(Vec3::Up());
		
		Vec3 tip_pt = Vec3::Up() * height * ((hit_y > 0) ? 1.0f : -1.0f);

		Vec3 m = origin - tip_pt;
		a = d.Dot(d);
		b = 2.0f * m.Dot(d);
		c = m.Dot(m) - (radius * radius);

		//discriminant
		discr = b * b - 4.0f * a * c;
		if (discr < 0.0f)return false;

		sqrt_d = VxSqrt(discr);
		t = (-b - sqrt_d) / (2.0f * a);

		if (t < 0.0f)
			t = (-b + sqrt_d) / (2.0f * a);
		if (t < 0.0f || t>1.0f)return false;
		hit.fraction = t;
		hit.normal = (m + d * t) / radius;


		return true;
	}


	VX_INLINE bool RayBox(const RayCast& local_cast, const Shape* shape, RaycastHit& hit)
	{
		const BoxShape* box = static_cast<const BoxShape*>(shape);

		float t_min = 0.0f;
		Float3 axis_min;
		float t_max = kMaxf;
		Vec3 inv_disp = local_cast.invDisplacement;

		Vec3 _min = box->GetLocalBounds().mMin;
		Vec3 _max = box->GetLocalBounds().mMax;
		if (Geometry::RayAABB(local_cast.origin, inv_disp, _min, _max, axis_min))
		{
			Vec3 _mins = Vec3::LoadFloat3Raw(axis_min);
			hit.fraction = _mins.MaxComponent();
			Axis axis = _mins.MaxAxis();
			Vec3 nor = Vec3::Zero();
			nor[int(axis)] = (inv_disp[int(axis)] < 0.0f) ? 1.0f : -1.0f;
			hit.normal = nor;
			return true;
		}
		return false;
	}



	WorldQuery::WorldQuery()
	{
		for (int i = 0; i < int(EShapeType::Count); ++i)
				mRayShapeDispatch[i] = &UnsupportedPair;


		mRayShapeDispatch[int(EShapeType::Sphere)] = &RaySphere;
		mRayShapeDispatch[int(EShapeType::Plane)] = &RayPlane;
		mRayShapeDispatch[int(EShapeType::Capsule)] = &RayCapsule;
		mRayShapeDispatch[int(EShapeType::Box)] = &RayBox;
	}

	bool WorldQuery::CastRay(const RayCast& ray_cast, QueryProcessor& query_processor)
	{
		VX_ASSERT_WARN_RETURN(mBroadphase, false, "Broadphase null");
		WorldQueryContext<RayCast, RayShapeFn> ctx
		{ 
			ray_cast, 
			query_processor,
			mRayShapeDispatch,
			(mDrawBroadphaseWalkedNodes) ? mDebugRender : nullptr
		};
		mBroadphase->CastRay(ray_cast, ctx);
		return ctx.HasHit();
	}



	bool WorldQuery::CastRay(const Vec3& origin, const Vec3& displacement, QueryProcessor& query_processor)
	{
		RayCast ray_cast = RayCast(origin, displacement);
		return CastRay(ray_cast, query_processor);
	}


}