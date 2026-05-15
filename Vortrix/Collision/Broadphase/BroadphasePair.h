#pragma once


namespace vx {
	class Body;
	struct BroadphasePair
	{
		Body* a = nullptr;
		Body* b = nullptr;

		BroadphasePair(Body* _a, Body* _b) :
			a(_a), b(_b) {
		}

		//Deterministic ordering (so A,B == B,A)
		bool operator==(const BroadphasePair& rhs) const noexcept
		{
			return (a == rhs.a && b == rhs.b) || (a == rhs.b && b == rhs.a);
		}


		struct Hash
		{
			size_t operator()(const BroadphasePair& p) const noexcept
			{
				auto a_id = reinterpret_cast<size_t>(p.a);
				auto b_id = reinterpret_cast<size_t>(p.b);
				return a_id ^ (b_id << 1);
			}
		};
	};
} //namespace vx