#pragma once

#include "NonCopyable.h"
#include "Core.h"
#include "Assertion.h"

#include "VxMemory.h"


namespace vx {

	class ScratchAllocator : public NonCopyable
	{
	public:
		explicit ScratchAllocator(size_t size) : mStackSize(size)
		{
			VX_ASSERT(size > 0);
			mMemStart = reinterpret_cast<uint8*>(VX_ALLOC(size));
		}

		~ScratchAllocator()
		{
			VX_ASSERT(mStackTop == 0, "Scratch allocator about to release, memory might still be used by other");
			VX_FREE(mMemStart);
		}
		
		[[nodiscard]] void* Allocate(size_t size)
		{
			if (size == 0)
				return nullptr;

			size_t new_stack_top = mStackTop + AlignUp(size, 16);
			if (new_stack_top > mStackSize)
			{
				VX_LOG_ERROR("Out of memory, Trying to allocate from scratch: ", size);
				return nullptr;
			}

			void* alloc_base = mMemStart + mStackTop;
			mStackTop = new_stack_top;

#if defined(VX_DEBUG_ALLOCATOR)
			mDebugTotalAlloc += size;
#endif // defined VX_DEBUG_ALLOCATOR

			return alloc_base;
		}

		void Free(void* mem_base, size_t size)
		{
			if (mem_base == nullptr)
			{
				VX_ASSERT_WARN(size == 0, "Attempting to free mem from scratch, which is null but has size");
				return; // pointer is null and size is zero
			}

			mStackTop -= AlignUp(size, 16);
			VX_ASSERT(mMemStart + mStackTop == mem_base, "Miss match, must have free mem in wrong order");
		}

		bool Empty() const { return mStackTop == 0; }
		size_t Size() const { return mStackSize; }
		size_t Usage() const { return mStackTop; }

		bool HasAddress(const void* addr) const
		{
			const uint8* _addr = reinterpret_cast<const uint8*>(addr);
			return _addr >= mMemStart && _addr < mMemStart + mStackSize;
		}

#if defined(VX_DEBUG_ALLOCATOR)
		void ResetDebugAlloc() { mDebugTotalAlloc = 0; }
		size_t DebugTotalAlloc() const { return mDebugTotalAlloc; }
#endif // defined VX_DEBUG_ALLOCATOR

	private:
		uint8* mMemStart;
		size_t mStackSize;
		size_t mStackTop = 0;

#if defined(VX_DEBUG_ALLOCATOR)
		size_t mDebugTotalAlloc = 0;
#endif // defined VX_DEBUG_ALLOCATOR

	};
} //namespace vx