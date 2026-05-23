#pragma once
#include "Core/Core.h"


namespace vx {
	class SolverBodyIndex
	{
	public:
		static constexpr uint32 kInvalidIndex = 0xffffffff;

		SolverBodyIndex() : mIndex(kInvalidIndex) {}
		explicit SolverBodyIndex(uint32 idx) : mIndex(idx) {}

		uint32 Value() const { return mIndex; }
		bool IsValid() const { return mIndex != kInvalidIndex; }

		VX_INLINE bool operator < (const SolverBodyIndex& rhs) { return mIndex < rhs.mIndex; }
		VX_INLINE bool operator > (const SolverBodyIndex& rhs) { return mIndex > rhs.mIndex; }

		bool operator == (const SolverBodyIndex& rhs) const { return mIndex == rhs.mIndex; }
		bool operator != (const SolverBodyIndex& rhs) const { return mIndex != rhs.mIndex; }
	private:
		uint32 mIndex = kInvalidIndex;
	};
} ///namespace vx