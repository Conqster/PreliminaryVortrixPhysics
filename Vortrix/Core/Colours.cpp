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

	Colour Colour::RandomColour(int idx)
	{
		return sRandomColourInst[idx % sRandomColourInst.size()];


		//uint32 h = idx * 2654435761u;

		//float r = ((h >> 16) & 255) / 255.0f;
		//float g = ((h >> 8) & 255) / 255.0f;
		//float b = ((h & 255) & 255) / 255.0f;

		//return Colour(r, g, b);
	}
}
