#pragma once


#include "ScalarMath.h"
#include "Vec3.h"

#include <stdlib.h>


namespace vx
{
	namespace Random
	{
		//quick random value between two value, min & max
		//even its float (its considered as a float point (which could be a float / double) 
		[[nodiscard]] static inline float Float(float min = 0.0, float max = 1.0)
		{
			return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
		}


		//quick random value between two value, min & max
		[[nodiscard]] static inline int Int(int min = 0, int max = 1)
		{
			return min + rand() / (RAND_MAX / (max - min));
		}


		//quick random point but not fully uniform 
		[[nodiscard]] static inline Vec3 PointInSphere(float radius)
		{
			if (radius <= 0.0f)
				return Vec3(0.0f);

			while (true)
			{
				float x = Float(-radius, radius);
				float y = Float(-radius, radius);
				float z = Float(-radius, radius);

				//printf("Random point in sphere x: %f, y: %f, z: %f\n", x, y, z);

				if (x * x + y * y + z * z <= radius * radius)
					return Vec3(x, y, z);
			}
		}
	}//Random namespace
}