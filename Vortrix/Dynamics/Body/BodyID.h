#pragma once

#include "Vortrix/Core/Core.h"
//#include "Vortrix/Body/Body.h"
namespace vx
{
	class BodyID
	{
	public:
		static constexpr uint32 kInvalidID = 0xffffffff;

		BodyID() : mID(kInvalidID){}
		explicit BodyID(uint32 id) : mID(id){}

		uint32 Value() const { return mID; }

		bool IsValid() const { return mID != kInvalidID; }
		[[nodiscard]] operator uint32() const { return mID; }

		VX_INLINE bool operator < (const BodyID& rhs) { return mID < rhs.mID; }
		VX_INLINE bool operator > (const BodyID& rhs) { return mID > rhs.mID; }

		bool operator == (const BodyID& rhs) const { return mID == rhs.mID; }
		bool operator != (const BodyID& rhs) const { return mID != rhs.mID; }
	private:
		uint32 mID = kInvalidID;
	};
}


