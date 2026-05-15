//#pragma once
//
//#include "Vortrix/Maths/Vec3.h"
//#include "Vortrix/Maths/Quat.h"
//
//namespace vx {
//
//
//	struct RaycastHit;
//	struct Ray
//	{
//		const Vec3& origin;
//		const Vec3& dir; //displacement to work with hitfraction not a unit direction
//
//
//		const Vec3& invDir; //or consterpt function
//	};
//	struct RaycastOutput  // or RaycastHit
//	{
//		float hitFraction;
//		vx::Vec3 normal;
//		//float distance; actuall could be deduced from fraction
//
//
//		BodyID bodyID;
//	};
//
//
//
//	//or 
//	struct ClosestHitRaycastProcessor //or BaseRaycastProcessor
//	{
//		//const Vec3& origin;
//		//const Vec3& dir; //displacement to work with hitfraction not a unit direction
//
//		float closestFraction = kMaxf;
//
//		RaycastOutput hits;
//	};
//
//	struct AllHitRaycastProcessor
//	{
//		float closestFraction = kMaxf;
//
//		RaycastOutput* hits;
//	};
//
//	
//
//
//
//
//	struct BroadphaseProxy
//	{
//		BodyID bodyID;
//		class Shape* mShape;
//		AABB bounds;
//	};
//
//	ctx.OnBroadphaseHit({ n.body, shape_type, ..... });
//
//
//	using RaycastFunc = bool(*)(const Shape*, const Vec3&, const Quat&, const Shape*, const Vec3&, const Quat&, ContactManifold&);
//
//	class Broadphase
//	{
//	public:
//
//		template<typename Context>
//		bool CastRay(const Ray& ray, Context& ctx);
//	};
//
//
//	template<typename Processor, typename CastFunc>
//	struct WorldQueryContext
//	{
//		const Ray& worldRay;
//		Processor& processor;
//		const CastFunc* dispatchTable;
//
//		bool OnBroadphaseHit(BroadphaseProxy proxy)
//		{
//
//		}
//
//		bool HasHit()
//		{
//			return processor.HasHit();
//		}
//
//		void AddHit()
//		{
//			processor.AddHit()
//		}
//	};
//
//	class WorldQuery
//	{
//	public:
//		template<typename QueryProcessor>
//		bool CastRay(const Ray& ray, QueryProcessor& processor)
//		{
//			WorldQueryContext<QueryProcessor, RaycastFunc> ctx{ ray, processor, mRaycastDispatch };
//			mBroadphase->CastRay(ray, ctx);
//
//			return ctx.HasHit();
//		}
//
//		template<typename Processor>
//		bool CastRay(const Vec3& origin, const Vec3& dir, Processor& processor);
//
//	private:
//		class Broadphase* mBroadphase;
//		RaycastFunc mRaycastDispatch[int(EShapeType::Count)];
//	};
//
//
//
//	template<typename Processor>
//	bool CastRay(const Ray& ray, Processor& processor)
//	{
//		struct Dispatcher //Accessor
//		{
//			bool Intersect(EShapeType type)
//			{
//				return mDispatcherTable[type](ray, ....)
//			}
//		};
//
//		mBroadphase->CastRay(.....)
//	}
//
//
//	struct ShapeSnapshot
//	{
//		Vec3 mPosition;
//		Quat mOrientation;
//		class Shape* mShape;
//	};
//
//
//	struct ShapeSnapshot
//	{
//		//RaycastHit/RaycasrResult
//		//CastRay action
//		bool CastRay(const Ray& ray, RaycastHit& io_hit) const;
//		bool CastRay(const Vec3& origin, const Vec3& direction, float max_dist, RaycastHit& io_hit) const;
//
//
//		struct RaycastResult
//		{
//			bool hadHit;
//			RaycastHit closestHit;
//		};
//
//		struct RaycastCollection
//		{
//			RaycastHit* hits;
//		};
//
//		//narrowphase query
//		bool CastRayClosest(const Ray& ray, RaycastHit& io_hit) const; // nearest hit
//		bool CastRayAny(const Ray& ray) const; //stop at first hit
//		bool CastRayAll(const Ray& ray, RaycastCollection& io_hit_results) const; //collect all hits
//
//
//
//
//		//ClosetestHitCollector
//		//AnyHitCollector
//		//AllHitsHitCollector
//		template<typename Collector>
//		void Raycast(const Ray&, Collector& collector) const;
//
//		Vec3 mPosition;
//		Quat mOrientation;
//		class Shape* mShape;
//	};
//}
//
//
