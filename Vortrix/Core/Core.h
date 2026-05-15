#pragma once

#include <memory>


#define EXPAND_MACRO(x) x

#if defined(_DEBUG)
	#define VX_DEBUG 1
	#define IMGUI_IMPL_OPENGL_DEBUG 1
#endif

#define VX_DEBUG_DRAW 1
#define VX_PROFILING 1
//#define VX_DISABLE_FORCE_INLINE


#define VX_STRINGIFY(x) #x

///Detect machine compiler 
#if defined(_MSC_VER)
#pragma message("_MSC_VER is defined (MSVC compiler). Value = " VX_STRINGIFY(_MSC_VER))
#elif defined(__clang__)
#pragma message("__clang__ is defined (Clang complier)")
#elif defined(__GNUC__)
#pragma message("__GNUC__ is defined (GCC complier)")
#else
#pragma message("Unknown compiler")
#endif // defined(_MSC_VER)


/// CPU arch
/// __x86_64 -> 64 bits x86   --- GCC/Clang
/// __i386/__i386__ 32 bits x86 -- GCC/Clang
/// __arm__ ARM 32 bit
/// __aarch64 ARM 64 bit
/// 
/// _M_IX86 -> 32-bit x86 -- MSVC
/// _M_X64 -> 64-bit x86 -- MSVC
/// _M_AMD64



#if defined(VX_DISABLE_FORCE_INLINE)
	#define VX_INLINE inline
#elif defined(_MSC_VER)
	#define VX_INLINE __forceinline
#else
	#define VX_INLINE inline __attribute__((always_inline))
#endif





#if defined(_MSC_VER)
	#define DEBUG_BREAK() __debugbreak()
#endif // defined(_MSC_VER)


#ifdef _WIN32

	#ifdef VPHX_SHARED_LIBRARY
		#ifdef VPHX_BUILD_SHARED_LIBRARY
			#define VX_API __declspec(dllexport)
		#else
			#define VX_API __declspec(dllimport)
		#endif // VPHX_BUILD_SHARED_LIBRARY
	#else
		#define VX_API 
	#endif // VPHX_SHARED_LIBRARY

#else
	#define VX_API __attribute__((visibility("default")))
#endif // _WIN32


#if defined(_MSC_VER)
	#define VX_PACK_PUSH(n) __pragma(pack(push, n))
	#define VX_PACK_POP __pragma(pack(pop))
#elif defined(__GNUC__) || defined(__clang__)
	#define VX_PACK_PUSH(n) _Pragma(VX_STRINGIFY(pack(push, n)))
	#define VX_PACK_POP _Pragma("pack(pop)")
#endif // defined(_MSC_VER)



//standard c++ includes
#include <vector>

namespace vx
{


	/// \brief safe and scoped pointer to an object. [Like unique_ptr]
	template<typename T>
	using Scope = std::unique_ptr<T>; 
	template<typename T, typename... Args>
	constexpr Scope<T> MakeScope(Args&& ...args) { return std::make_unique<T>(std::forward<Args>(args)...); }

	/// \brief ref counted pointer to an object. [Usually used for resources]
	template<typename T>
	using Ref = std::shared_ptr<T>;
	template<typename T, typename... Args>
	constexpr Ref<T> MakeRef(Args&& ...args) { return std::make_shared<T>(std::forward<Args>(args)...); }


	template<typename T>
	inline constexpr T Bit(unsigned x) { return static_cast<T>(T(1) << x); } 

	inline constexpr uint32_t Bit32(unsigned x) { return Bit<uint32_t>(x); } 
	inline constexpr uint16_t Bit16(unsigned x) { return Bit<uint16_t>(x); }
	inline constexpr uint8_t Bit8(unsigned x) { return Bit<uint8_t>(x); }

	using uint = unsigned int;
	using uint8 = uint8_t;
	using uint16 = uint16_t;
	using uint32 = uint32_t;
	using uint64 = uint64_t;

	using int8 = int8_t;
	using int16 = int16_t;
	using int32 = int32_t;
	using int64 = int64_t;

	//to prevent truncation of 64-bit pointer to 32-bit
	using intptr = intptr_t; //<-- sweet for bitwise ops (address math, masking, hashing)
	using uintptr = uintptr_t; //<-- sweet for numerical interpretation (differences, offsets).


	//template<typename T>
	//inline constexpr T Clamp(T value, T min_val, T max_val)  {  return std::min(std::max(value, min_val), max_val); }

	//TODO(Conqster): later use a build system CMAKE for PROJECT_SOURCE_DIR rather than in code
//#define PROJECT_SOURCE_DIR "C:\\Users\\okeja\\Desktop\\Personal Projects\\PhysicsEngineRendering\\"

	inline const char* StripProjectPath(const char* file_path)
	{
#if defined(PROJECT_SOURCE_DIR)
		if (const char* base = std::strstr(file_path, PROJECT_SOURCE_DIR))
			return base + std::strlen(PROJECT_SOURCE_DIR);
#endif // defined(PROJECT_SOURCE_DIR)


		return file_path;
	}


	template<typename T, typename CompareT>
	static void InsertSort(T* arr, int left, int right, CompareT compare)
	{
		int first = left;
		int last = right;

		if (left != right)
		{
			for (int i = left + 1; i <= right; ++i)
			{
				//copy value before corrupt
				T x = std::move(arr[i]);

				//less than first 
				if (compare(arr[i], arr[first]))
				{
					//move up 
					for (int j = i; j != first; --j) //and tranvers backwards 
					{
						//swap is bad, going to ping pong data 
						arr[j] = arr[j - 1];
					}

					//move resident back to new location
					arr[first] = std::move(x);
				}
				else
				{
					int w = i; //stand by to look ahead first
					for (int j = w - 1; j >= first && compare(x, arr[j]); w = j, --j)
						arr[w] = std::move(arr[j]);

					arr[w] = std::move(x);
				}
			}
		}
	}
	
	///Quick sort 
	template<typename T, typename CompareT>
	static void QuickSort(T* arr, int left, int right, CompareT compare)
	{
		//constexpr int threshold = 64 / sizeof(T);
		if ((right - left) < 32)
			return InsertSort(arr, left, right, compare);


		int i = left;
		int j = right;

		const T& pivot = arr[(left + right) / 2];

		while (i <= j)
		{
			//first element that is bigger than pivot 
			while (compare(arr[i], pivot)) ++i;
			while (compare(pivot, arr[j])) --j;

			if (i <= j)
			{
				std::swap(arr[i], arr[j]);
				++i;
				--j;
			}
		}


		if (left < j)
			QuickSort(arr, left, j, compare);
		if(i < right)
			QuickSort(arr, i, right, compare);
	}

} // VPHX namespace


#undef uint16_t
