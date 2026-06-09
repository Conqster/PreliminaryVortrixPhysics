#include "Colours.h"

#include <array>
namespace vx {


	const static std::array<Colour, 32> GenerateRandomColours()
	{
		std::array<Colour, 32> t;
		for (auto& col : t)
		{
			col = Colour(
				Random::Float(0.0f, 1.0f), 
				Random::Float(0.0f, 1.0f), 
				Random::Float(0.0f, 1.0f));
		}

		return t;
	}

	const static std::array<Colour, 32> sRandomColourInst = GenerateRandomColours();

	Colour Colour::GetRandomColour(int idx)
	{
		return sRandomColourInst[idx % sRandomColourInst.size()];
	}
}
