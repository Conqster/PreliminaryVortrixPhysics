#pragma once
#include <atomic>

namespace vx::atomic {

	template<typename T>
	bool Min(std::atomic<T>& atomic, const T value,
		std::memory_order mem_order = std::memory_order_relaxed)
	{
		T curr_v = atomic.load(std::memory_order_relaxed);
		while (curr_v > value)
		{
			if (atomic.compare_exchange_weak(
				curr_v, value, mem_order,
				std::memory_order_relaxed))
				return true;
		}
		return false;
	}

	template<typename T>
	bool Max(std::atomic<T>& atomic, const T value,
		std::memory_order mem_order = std::memory_order_relaxed)
	{
		T curr_v = atomic.load(std::memory_order_relaxed);
		while (curr_v < value)
		{
			if (atomic.compare_exchange_weak(
				curr_v, value, mem_order,
				std::memory_order_relaxed))
				return true;
		}
		return false;
	}

}