#include "Mat44.h"

std::ostream& vx::operator<<(std::ostream& os, const Mat44& m)
{
	char buff[48];
	constexpr const char* format = "| %.3f %.3f %.3f %.3f |\n";

	std::string temp;
	temp.reserve(48 * 4);

	//snprintf(buff, sizeof(buff), format, n[i][0], n[i][1], n[i][2], n[i][3]);
	for (uint16_t row = 0; row < 4; ++row)
	{
		//[column][row]
		snprintf(buff, sizeof(buff), format,
			m[0][row], m[1][row], m[2][row], m[3][row]);

		temp += buff;
	}
	//return std::string(buff);

	//print linear 
	temp += "\n| ";
	for (size_t i = 0; i < 16; ++i)
	{
		const bool nxt_col = ((i % 4) == 0 && i != 0);
		if (nxt_col)
			snprintf(buff, sizeof(buff), ", %.3f", m.mFloats[i]);
		else
			snprintf(buff, sizeof(buff), " %.3f", m.mFloats[i]);

		temp += buff;
	}
	temp += " |.\n";
    //for (const auto& c : m.mCol)
    //    os << "\n|" << c.X() << " " << c.Y() << " " << c.Z() << " " << c.W() << "|";

    os << temp;
    return os;
}
