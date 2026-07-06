#pragma once
#include "Vortrix/Core/Core.h"
#include "Body/BodyID.h"

#include "Vortrix/Maths/Vec3.h"

namespace vx {
	class SolverBodyIndex
	{
	public:
		static constexpr uint32 kInvalidIndex = 0xffffffff;

		SolverBodyIndex() = default;
		explicit SolverBodyIndex(uint32 idx) : mIndex(idx) {}

		SolverBodyIndex Invalid()
		{
			SolverBodyIndex t;
			t.mIndex = kInvalidIndex;
			return t;
		}

		uint32 Value() const { return mIndex; }
		bool IsValid() const { return mIndex != kInvalidIndex; }

		VX_INLINE bool operator < (const SolverBodyIndex& rhs) { return mIndex < rhs.mIndex; }
		VX_INLINE bool operator > (const SolverBodyIndex& rhs) { return mIndex > rhs.mIndex; }

		bool operator == (const SolverBodyIndex& rhs) const { return mIndex == rhs.mIndex; }
		bool operator != (const SolverBodyIndex& rhs) const { return mIndex != rhs.mIndex; }
	private:
		uint32 mIndex;
	};
	static_assert(std::is_trivial_v<SolverBodyIndex>, "SolverBodyIndex must be a trivial type!");

	struct SolverBody
	{
		/// linear velocity
		Vec3 v = Vec3(0.0f);
		/// angularVelocity
		Vec3 w = Vec3(0.0f);
		float invMass;
		BodyID bodyID;
	};

} ///namespace vx