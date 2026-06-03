#include "Core.h"
#include "Core/Logger.h"

namespace vx{


	///Bytes -> kB
	double ToKilobyte(uint32_t bytes)
	{
		return static_cast<double>(bytes) / 1000;
	}
	///Bytes -> KiB
	double ToKibibyte(uint32_t bytes)
	{
		return static_cast<double>(bytes) / 1024;
	}

	///Bytes -> MB
	double ToMegabyte(uint32_t bytes)
	{
		return static_cast<double>(bytes) * 1e-6;
	}
	///Bytes -> MiB
	double ToMebibyte(uint32_t bytes)
	{
		return static_cast<double>(bytes) / (1024 * 1024);
	}

	struct MemoryProfile
	{
		/// allocation call count
		uint32_t allocs = 0;
		/// deallocation call count
		uint32_t deallocs = 0;
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

			//h->ptr = (void*)(h + 1);
			h->raw = h;
			h->size = size;
			return (void*)(h + 1);


			//mAllocations[mCurrTracking++] = *h;
			//return h->ptr;
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
			uintptr_t aligned_address = (base_address + uintptr_t(alignment - 1)) & ~(uintptr_t(alignment - 1));

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


		~MemoryProfile()
		{
			//VX_LOG_INFO("----------------------------OUT OF SCOPE ~ ------------------");
			//VX_LOG_INFO("Allocated Count: ", allocs);
			//VX_LOG_INFO("Allocated Bytes: ", allocatedBytes);
			//VX_LOG_INFO("Deallocated Count: ", deallocs);
			//VX_LOG_INFO("Deallocated Bytes: ", deallocatedBytes);
			//VX_LOG_INFO("Curr Allocated Count: ", CurrentAllocCount());
			//VX_LOG_INFO("Curr Allocated Bytes: ", CurrentAllocBytes());
			//VX_LOG_INFO("-------------------------------------------------------------");
		}

	};


	static MemoryProfile sMemoryProfile;

#define PROFILE_MEM_ALLOC 1


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


VX_INLINE void* operator new(size_t size) { return VX_ALLOC(size); }
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
