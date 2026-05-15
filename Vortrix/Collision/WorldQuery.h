#pragma once


#include "Vortrix/Maths/Vec4.h"
#include "Vortrix/Dynamics/Body/BodyID.h"


#include "Shapes/Shape.h"

class DebugGizmosRenderer;

namespace vx {



	using RayShapeFn = bool(*)(const struct RayCast&, const class Shape*, struct RaycastHit&);
	//struct SegmentCast
	//{

	//};

	
	struct RaycastHit
	{
		BodyID body;
		Vec3 normal;
		float fraction = 1.0f;
	};



	struct RaycastResult
	{
		RaycastHit* hits;
		uint32 hitCount;
		//uint32 maxHits;
	};

	class QueryProcessor
	{
	public:
		virtual bool AddHit(const RaycastHit& hit) = 0;
		virtual bool HasHit() const = 0;

		virtual bool ShouldEarlyOut() const = 0;
		virtual float EarlyOutFraction() const = 0;

		virtual RaycastResult Result() = 0;

		virtual ~QueryProcessor() = default;
	};

	class ClosestRaycastHitProcessor : public QueryProcessor
	{
	public: 
		bool AddHit(const RaycastHit& hit) override
		{
			if(hit.fraction < mClosest.fraction)
			{
				mClosest = hit;
				mHasHit = true;
			}
			return true;
		}

		bool HasHit() const override { return mHasHit; }
		bool ShouldEarlyOut() const override { return mClosest.fraction <= 0.0f; }
		float EarlyOutFraction() const override { return mClosest.fraction; }

		RaycastResult Result()  override
		{
			return mHasHit ? RaycastResult{&mClosest, 1} 
			: RaycastResult{ nullptr, 0 };
		}

		RaycastHit Hit() { return mClosest; }
	private:
		RaycastHit mClosest{ /*.fraction = 1.0f */};
		bool mHasHit = false;
	};

	class AnyRaycastHitProcessor : public QueryProcessor
	{
	public:
		bool AddHit(const RaycastHit& hit) override
		{
			if (hit.fraction < mClosest.fraction)
			{
				mClosest = hit;
				mHasHit = true;
			}
			return false;
		}

		bool HasHit() const override { return mHasHit; }
		//bool ShouldEarlyOut() const override { return mClosest.fraction <= 0.0f; }
		bool ShouldEarlyOut() const override { return mHasHit; }
		//no shrinking
		float EarlyOutFraction() const override { return 1.0f; }

		RaycastResult Result()  override
		{
			return mHasHit ? RaycastResult{ &mClosest, 1 }
			: RaycastResult{ nullptr, 0 };
		}

		RaycastHit Hit() { return mClosest; }
		RaycastHit* Hits() { return &mClosest; }
	private:
		RaycastHit mClosest{ /*.fraction = 1.0f */ };
		bool mHasHit = false;
	};


	template<size_t MaxHits = 16>
	class AllRaycastHitProcessor : public QueryProcessor
	{//
	public:
		bool AddHit(const RaycastHit& hit) override
		{
			if(mCount < MaxHits)
			{
				mHits[mCount++] = hit;
				mHasHit = true;
			}
			return mCount < MaxHits;
		}

		bool HasHit() const override { return mHasHit; }
		//bool ShouldEarlyOut() const override { return mClosest.fraction <= 0.0f; }
		bool ShouldEarlyOut() const override { return mCount >= MaxHits; }
		//float EarlyOutFraction() const override { return mClosest.fraction; }
		float EarlyOutFraction() const override { return 1.0f; }

		void Sort()
		{
			if (mCount == 0)
				return;
			QuickSort(mHits, 0, mCount - 1, [](const RaycastHit& lhs, const RaycastHit& rhs)
				{
					return lhs.fraction < rhs.fraction;
				});
		}

		RaycastResult Result()  override
		{
			return mHasHit ? RaycastResult{ mHits, mCount }
			: RaycastResult{ nullptr, 0 };
		}

		RaycastHit Hit() { return (mHasHit)?mHits[0] : RaycastHit(); }
		RaycastHit* Hits() { return mHits; }
	private:
		uint32 mCount = 0;
		bool mHasHit = false;
		RaycastHit mHits[MaxHits];
	};

	struct RayCast;

	class WorldQuery
	{
	public:

		WorldQuery();
		void Init(class Broadphase* broadphase)
		{
			mBroadphase = broadphase;
		}

		bool CastRay(const RayCast& ray_cast, QueryProcessor& query_processor);

		VX_INLINE bool CastRay(const Vec3& origin, const Vec3& displacement, QueryProcessor& query_processor);



		void SetDebugRender(DebugGizmosRenderer* renderer) { mDebugRender = renderer; }
		void SetDrawBroadphaseNodesWalked(bool v) { mDrawBroadphaseWalkedNodes = v; }

		/// use case 
		/// ClosestRaycastHitProcessor processor;
		/// worldQuery.CastRay(ray, processor);
		/// 
		/// RaycastResult result = processor.Result();
		/// 
		/// if(result.HasHit())
		/// {
		///		RaycastHit hit = result.hits[0]
		///		Body body = mPhysicsWorld.BodyManager().GetBody(hit.body);
		///		Vec3 hit_nor = hit.normal;
		///		Vec3 hit_position = ray_cast.origin + 
		///					ray_cast.displacement * hit.fraction;
		/// }
		/// 
		/// 
		/// also 
		/// if(processor.HasHit())
		/// {
		///		RaycastResult result = processor.Result();
		///		or since has hit check 
		///		RaycastHit hit = processor.GetHit() //return first hit "assume sorted" for closest any could be rando
		///		or 
		///		RaycastHit* hits = processor.GetHits();
		///
		///		
		/// }
	private:
		Broadphase* mBroadphase = nullptr;
		RayShapeFn mRayShapeDispatch[int(EShapeType::Count)];
		DebugGizmosRenderer* mDebugRender = nullptr;
		bool mDrawBroadphaseWalkedNodes = false;
	};
}