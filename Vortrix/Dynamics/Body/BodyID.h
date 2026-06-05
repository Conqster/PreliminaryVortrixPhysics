#pragma once

#include "Vortrix/Core/Core.h"
//#include "Vortrix/Body/Body.h"
namespace vx
{


	/// Body is inspired on Jolt Physics BodyID -> idx sequence
	/// BodyID -> 32 bits [0000|0000][0000|0000][0000|0000][0000|0000]
	/// generation bits | body vector idx bits
	/// lower 24 bits 0x00ffffff stores the idx of body in body manager bodies vector
	/// upper 8 bits 0xff000000, store id generation. to track and prevent
	/// collsion if after a body is remove a new body take the idx
	/// generation value range 0 - 255
	/// idx ranges 0 - 16,777,215
	/// 
	/// i.e mask for idx ->  0x00ffffff
	class BodyID
	{
	public:
		static constexpr uint32 kInvalidID = 0xffffffff;
		/// bit shift to for the upper 8 bits of 32 bits
		static constexpr uint32 kBitShiftGen = 24;
		static constexpr uint32 kIdxMask = 0x00ffffff;

		BodyID() : mID(kInvalidID){}
		explicit BodyID(uint32 id) : mID(id){}

		explicit BodyID(uint32 idx, uint8 generation) :
			mID((uint32(generation) << kBitShiftGen) | idx)
		{}

		uint32 ID() const { return mID; }
		uint8 Generation() const { return uint8(mID >> kBitShiftGen); }
		/// mask out generation from id 
		/// mID -> 0xyyyyyyyy
		/// y -> value
		/// mask -> 0x00ffffff
		/// mask & mID -> 0x00yyyyyy 
		uint32 Idx() const { return mID & kIdxMask; }

		bool IsValid() const { return mID != kInvalidID; }
		//[[nodiscard]] operator uint32() const { return mID; } <<- remove so to not mix id with idx

		VX_INLINE bool operator < (const BodyID& rhs) { return mID < rhs.mID; }
		VX_INLINE bool operator > (const BodyID& rhs) { return mID > rhs.mID; }

		bool operator == (const BodyID& rhs) const { return mID == rhs.mID; }
		bool operator != (const BodyID& rhs) const { return mID != rhs.mID; }
	private:
		uint32 mID = kInvalidID;
	};
}


