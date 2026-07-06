#pragma once

#include "Vortrix/Collision/Shapes/Shape.h"
#include "Vortrix/Collision/ContactManifold.h"

#include "Vortrix/Core/Logger.h"

namespace vx {

	using CollisionFn = bool(*)(const Shape*, const Vec3&, const Quat& , const Shape*, const Vec3&, const Quat&, ContactManifold&);


	class CollisionDispatcher
	{
	public:

		CollisionDispatcher()
		{

			for (int i = 0; i < int(EShapeType::Count); i++)
				for (int j = 0; j < int(EShapeType::Count); j++)
					mDispatchTable[i][j] = &UnsupportedPair;
		}

		VX_INLINE CollisionFn Get(EShapeType a, EShapeType b) const
		{
			return mDispatchTable[static_cast<size_t>(a)][static_cast<size_t>(b)];
		}

		VX_INLINE void Register(EShapeType a, EShapeType b, CollisionFn func)
		{
			mDispatchTable[int(a)][int(b)] = func;
		}



		template<CollisionFn Func>
		static bool SwappedRef(
			const Shape* a, const Vec3& in_posA, const Quat& in_orienA,
			const Shape* b, const Vec3& in_posB, const Quat& in_orienB,
			ContactManifold& manifold)
		{
			if (Func(b, in_posB, in_orienB,
				a, in_posA, in_orienA,
				manifold))
			{
				manifold.Flip();
				return true;
			}
			return false;
		}

	private:
		CollisionFn mDispatchTable[int(EShapeType::Count)][int(EShapeType::Count)];

		static bool UnsupportedPair(const Shape* a, const Vec3& p0, const Quat& q0, 
			const Shape* b, const Vec3& p1, const Quat& q1,
			ContactManifold&) 
		{ 
			VX_LOG_WARN("Unsupported Dispatch Pairs {", a->GetShapeTypeName(), "-", b->GetShapeTypeName(), "}!!!"); 
			return false; 
		}
	};
}