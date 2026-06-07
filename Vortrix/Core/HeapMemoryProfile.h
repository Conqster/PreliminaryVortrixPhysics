#pragma once

#include "Core.h"

namespace vx {

	struct MemoryProfile
	{
		/// allocation call count
		uint32 allocs = 0;
		/// deallocation call count
		uint32 deallocs = 0;
		/// total allocated bytes 
		size_t allocatedBytes = 0;
		/// total deallocated bytes 
		size_t deallocatedBytes = 0;

		size_t CurrentAllocBytes() const { return allocatedBytes - deallocatedBytes; }
		size_t CurrentAllocCount() const { return allocs - deallocs; }

		struct Header
		{
			void* raw;
			size_t size;
		};

		VX_INLINE void* Alloc(size_t size)
		{
			size_t _size = sizeof(Header) + size;

			/// using actual size, for debugging 
			/// so user would be confuse about extra allocation
			allocatedBytes += size;
			allocs++;

			Header* h = (Header*)malloc(_size);

			h->raw = h;
			h->size = size;
			return (void*)(h + 1);
		}
		/// See PhysX Memory allocatir allocAligned
		VX_INLINE void* AlignedAlloc(size_t size, size_t alignment)
		{
			///static_assert(size > 0 && alignment > 0);

			/// using actual size, for debugging 
			/// so user would be confuse about extra allocation
			allocatedBytes += size;
			allocs++;

			size_t total = sizeof(Header) + size + alignment - 1;

#ifdef  _WIN32
			void* raw = _aligned_malloc(total, alignment);
#else
			void* raw = std::aligned_alloc(alignment, total);
#endif //  _WIN32

			uintptr_t base_address = uintptr_t(raw) + sizeof(Header);
			uintptr_t aligned_address = AlignUp(base_address, alignment);

			Header* h = (Header*)(aligned_address - sizeof(Header));

			h->raw = raw;
			h->size = size;
			return (void*)aligned_address;
		}
		VX_INLINE void Dealloc(void* block)
		{
			//void* deallo_block = block;
			if (!block)return;

			Header* h = ((Header*)block) - 1;
			deallocatedBytes += h->size;
			deallocs++;
			free(h->raw);
		}
		VX_INLINE void AlignedDealloc(void* block)
		{
			if (!block)return;

			Header* h = (Header*)(uintptr_t(block) - sizeof(Header));
			deallocatedBytes += h->size;
			deallocs++;

#ifdef  _WIN32
			_aligned_free(h->raw);
#else
			free(h->raw);
#endif //  _WIN32
		}
	};

#if PROFILE_MEM_ALLOC
	extern MemoryProfile sMemoryProfile;
#endif // PROFILE_MEM_ALLOC

} //namespace vx