#pragma once
#include <unordered_map>
#include "Core/Core.h"
#include "Core/NonCopyable.h"


namespace vx {


	template<typename Key_T>
	struct DefaultMapHasher
	{
		size_t operator()(const Key_T& k) const noexcept
		{
			return k.Hash();
		}
	};

	////https://codeforces.com/blog/entry/144477
	///
	////https://en.cppreference.com/cpp/memory/null_memory_resource
	////https://www.reddit.com/r/cpp/comments/jf0dse/performance_of_stdpmr/

	/// Similar to jolt, but wrapper around std unordered 
	/// for work around multithreading and woulkd love to 
	/// implement a hash map myself
	/// 
	/// this tries to bypass internal hashing 
	/// so has to share hash value with physics syste, like 
	/// instance in different Map abd with different key 
	/// using ManifoldMap = HashMap<BodyIDPairPoint, CachedManifold>;
	/// using BodyPairMap = HashMap<BodyPair, CachedBodyPair>;
	/// should still be able to shared hash value
	template <class _Key, class _Value, class Hasher = DefaultMapHasher<_Key>>
	class HashMap : public NonCopyable
	{
	public:
		//using MapType = std::unordered_map<_Key, _Value, Hasher>;
		using MapType = std::pmr::unordered_map<_Key, _Value, Hasher>;
		///using MapEntry = 

		HashMap()
		{
			/// to prevent division
			///project_load = float(map.size() + 1) / map.bucket_count()
			mMap.max_load_factor(1.0f);
		}

		VX_INLINE void Clear() { mMap.clear(); }

		void Init(uint32 max_bucket)
		{
			mMap.rehash(max_bucket);
			//mMap.reserve(max_bucket);
		}

		class Entry
		{
		public:
			bool Valid() const { return mValue != nullptr; }
			const _Key& Key() const { return *mKey; }
			_Value& Value() const { return *mValue; }
		private:
			friend class HashMap;
			const _Key* mKey = nullptr;
			_Value* mValue = nullptr;
		};

		//could use MapType::it


		using Iterator = typename MapType::iterator;
		_NODISCARD Iterator begin() noexcept { return mMap.begin(); }
		_NODISCARD Iterator end() noexcept { return mMap.end(); }

		template<class... Args>
		VX_INLINE Entry Create(const _Key& _key, /*uint64 key_hash, */ Args&&... args)
		{
			Entry e;
			if ((mMap.size() + 1) >= mMap.bucket_count())
			{
				VX_ASSERT_WARN(false, "Create rejected, new entry will cause rehashing");
				return e;
			}

			auto [it, inserted] =
				mMap.try_emplace(_key, std::forward<Args>(args)...);

			e.mKey = &it->first;
			e.mValue = &it->second;
			return e;
		}


		VX_INLINE Entry Find(const _Key& key/*, uint64 key_hash*/)
		{
			auto it = mMap.find(key);

			Entry e;
			if (it == mMap.end())
				return e;

			e.mKey = &it->first;
			e.mValue = &it->second;
			return e;
		}

	private:
		MapType mMap;
	};
} ///namespace vx