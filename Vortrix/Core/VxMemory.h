#pragma once
#include "Core.h"

namespace vx{

	VX_INLINE void* Allocate(size_t size)
	{
		//static_assert(size > 0);
		return malloc(size);
	}
	VX_INLINE void* AlignedAllocate(size_t size, size_t alignment)
	{
		///static_assert(size > 0 && alignment > 0);
#ifdef  _WIN32
		return _aligned_malloc(size, alignment);
#else
		return std::aligned_alloc(align, size);
#endif //  _WIN32

	}
	VX_INLINE void Deallocate(void* block)
	{
		free(block);
	}
	VX_INLINE void AlignedDeallocate(void* block)
	{
#ifdef  _WIN32
		_aligned_free(block);
#else
		free(block);
#endif //  _WIN32
	}

} //namespace vx


#if PROFILE_MEM_ALLOC
#include "HeapMemoryProfile.h"
#define VX_ALLOC(x) vx::sMemoryProfile.Alloc(x)
#define VX_ALIGN_ALLOC(size, align) vx::sMemoryProfile.AlignedAlloc(size, align)

#define VX_FREE(x) vx::sMemoryProfile.Dealloc(x)
#define VX_ALIGN_FREE(x) vx::sMemoryProfile.AlignedDealloc(x)
#else
#define VX_ALLOC(x) vx::Allocate(x)
#define VX_ALIGN_ALLOC(size, align) vx::AlignedAllocate(size, align)

#define VX_FREE(x) vx::Deallocate(x)
#define VX_ALIGN_FREE(x) vx::AlignedDeallocate(x)
#endif // !PROFILE_MEM_ALLOC


VX_INLINE void* operator new (size_t size) { return VX_ALLOC(size); }
VX_INLINE void operator delete (void* pointer) noexcept { VX_FREE(pointer); }
VX_INLINE void operator delete(void* pointer, [[maybe_unused]] size_t size) noexcept { VX_FREE(pointer); }

VX_INLINE void* operator new[](size_t size) { return VX_ALLOC(size); }
VX_INLINE void operator delete[](void* pointer) noexcept { VX_FREE(pointer); }
VX_INLINE void operator delete[](void* pointer, [[maybe_unused]] size_t size) noexcept { VX_FREE(pointer); }

VX_INLINE void* operator new(size_t size, std::align_val_t alignment) { return VX_ALIGN_ALLOC(size, static_cast<size_t>(alignment));}
VX_INLINE void operator delete(void* pointer, [[maybe_unused]] std::align_val_t align) noexcept { VX_ALIGN_FREE(pointer); }
VX_INLINE void operator delete(void* pointer, size_t size, [[maybe_unused]] std::align_val_t align) noexcept { VX_ALIGN_FREE(pointer); }


VX_INLINE void* operator new[](size_t size, std::align_val_t alignment) noexcept { return VX_ALIGN_ALLOC(size, static_cast<size_t>(alignment)); }
VX_INLINE void operator delete[](void* pointer, [[maybe_unused]] size_t size, [[maybe_unused]] std::align_val_t align) noexcept { VX_ALIGN_FREE(pointer); }
VX_INLINE void operator delete[](void* pointer, [[maybe_unused]] std::align_val_t align) noexcept { VX_ALIGN_FREE(pointer); }
