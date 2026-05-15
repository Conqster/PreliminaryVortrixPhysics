#pragma once

#include "Vortrix.h"

namespace vx 
{

	VX_INLINE uint32 PackRGBA(float r, float g, float b, float a)
	{
		r = VxClamp01(r);
		g = VxClamp01(g);
		b = VxClamp01(b);
		a = VxClamp01(a);

		uint8 _r = static_cast<uint8>(r * 255.0f);
		uint8 _g = static_cast<uint8>(g * 255.0f);
		uint8 _b = static_cast<uint8>(b * 255.0f);
		uint8 _a = static_cast<uint8>(a * 255.0f);
		return  (static_cast<uint32>(_a) << 24) |
				(static_cast<uint32>(_b) << 16) |
				(static_cast<uint32>(_g) << 8) |
				static_cast<uint32>(_r);
	}

	VX_INLINE uint32 PackRGBA(float r, float g, float b) 
	{
		return PackRGBA(r, g, b, 1.0f);
	}

	VX_INLINE uint32 PackRGBA(const Vec4& c)
	{
		return PackRGBA(c[0], c[1], c[2], c[3]);
	}

	VX_INLINE uint32 PackRGBA(const Vec3& c, float a = 1.0f) 
	{
		return PackRGBA(c[0], c[1], c[2], a);
	}

	VX_INLINE uint32 PackRGBA(float c) 
	{
		return PackRGBA(c, c, c, c);
	}

	VX_INLINE Vec3 RGBAAsVec3(uint32 rgba)
	{
		//colour channels 
		uint8 r = (rgba & 0x000000FF);
		uint8 g = (rgba & 0x0000FF00) >> 8;
		uint8 b = (rgba & 0x00FF0000) >> 16;

		return Vec3(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b)) / 255.0f;
	}

	VX_INLINE Vec4 RGBAAsVec4(uint32 rgba)
	{
		//colour channels 
		uint8 r = (rgba & 0x000000FF);
		uint8 g = (rgba & 0x0000FF00) >> 8;
		uint8 b = (rgba & 0x00FF0000) >> 16;
		uint8 a = (rgba & 0xFF000000) >> 24;

		return Vec4(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), static_cast<float>(a)) / 255.0f;
	}

	VX_INLINE Vec3 RGBAsVec3(uint8 r, uint8 g, uint8 b)
	{
		return Vec3(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b)) / 255.0f;
	}

	VX_INLINE Vec4 RGBAAsVec4(uint8 r, uint8 g, uint8 b, uint8 a)
	{
		return Vec4(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), static_cast<float>(a)) / 255.0f;
	}

	struct [[nodiscard]] Colour
	{
		union {
			uint32 value;
			struct {
				//depending on development r might be promoted to most importance
				uint8 r; //Little endiann least significat byte
				uint8 g;
				uint8 b;
				uint8 a; //Most significant byte 
			};
		};


		explicit Colour() : value(0) {}
		explicit Colour(float _v) : value(PackRGBA(_v, _v, _v, 1.0f)) {}
		/*explicit*/ Colour(float _r, float _g, float _b, float _a = 1.0f) : value(PackRGBA(_r, _g, _b, _a)) {}
		explicit Colour(uint8 red_channel, uint8 green_channel, uint8 blue_channel, uint8 alpha_channel = 255) : 
			r(red_channel), g(green_channel), b(blue_channel), a(alpha_channel) {}
		explicit Colour(uint32 v_col) : value(v_col) {}
		explicit Colour(const Vec3& rgb, float _a = 1.0f) : value(PackRGBA(rgb, _a)) {}
		explicit Colour(const Vec4& rgba) : value(PackRGBA(rgba)) {}

		//Convert to glm
		[[nodiscard]] Vec3 ToVec3() const { return RGBAsVec3(r, g, b); }
		[[nodiscard]] Vec4 ToVec4() const { return RGBAAsVec4(r, g, b, a); }



		//implicit conversions
		// to packed uint32_t
		[[nodiscard]] operator uint32() const { return value; }
		[[nodiscard]] operator Vec3() const { return ToVec3(); }
		[[nodiscard]] operator Float3() const { return ToVec3().ToFloat3(); }
		[[nodiscard]] operator Vec4() const { return ToVec4(); }

		bool operator==(const Colour& other) const { return value == other.value; }
		bool operator!=(const Colour& other) const { return value != other.value; }

		void ToColour(const float* ptr_vec, bool has_alpha = false)
		{
			r = static_cast<uint8>(ptr_vec[0] * 255.0f);
			g = static_cast<uint8>(ptr_vec[1] * 255.0f);
			b = static_cast<uint8>(ptr_vec[2] * 255.0f);
			if(has_alpha)
				a = static_cast<uint8>(ptr_vec[3] * 255.0f);
		}

		//channel accessing
		inline const uint8& operator[](uint channel) const
		{
			VX_ASSERT_WARN(channel < 4, "Trying to access invalid channel corrdinate");

			switch (channel)
			{
			case 0: return r;
			case 1: return g;
			case 2: return b;
			case 3: return a;
			default: return a;
			}
		}

		VX_INLINE float R() const { return static_cast<float>(r) / 255.0f; }
		VX_INLINE float G() const { return static_cast<float>(g) / 255.0f; }
		VX_INLINE float B() const { return static_cast<float>(b) / 255.0f; }
		VX_INLINE float A() const { return static_cast<float>(a) / 255.0f; }


		static const Colour sWhite;
		static const Colour sBlack;
		static const Colour sRed;
		static const Colour sGreen;
		static const Colour sDeepTeal;
		static const Colour sBlue;
		static const Colour sTurquoise;
		static const Colour sYellow;
		static const Colour sOrange;
		static const Colour sCyan;
		static const Colour sMagenta;
		static const Colour sPurple;
	};

	inline const Colour Colour::sWhite(1.0f);
	inline const Colour Colour::sBlack(0.0f);
	inline const Colour Colour::sRed(1.0f, 0.0f, 0.0f);
	inline const Colour Colour::sGreen(0.0f, 1.0f, 0.0f);
	inline const Colour Colour::sDeepTeal(0.0f, 0.3f, 0.35f);
	inline const Colour Colour::sBlue(0.0f, 0.0f, 1.0f);
	inline const Colour Colour::sTurquoise(0.25f, 0.88f, 0.82f);
	inline const Colour Colour::sYellow(1.0f, 1.0f, 0.0f);
	inline const Colour Colour::sOrange(1.0f, 0.647f, 0.0f);
	inline const Colour Colour::sCyan(0.0f, 1.0f, 1.0f);
	inline const Colour Colour::sMagenta(1.0f, 0.0f, 1.0f);
	inline const Colour Colour::sPurple(0.5f, 0.0f, 0.5f);

	//future if needed
	//struct ColourRGBA16
	//{
	//	union {
	//		uint64_t value;
	//		struct {
	//			//depending on development r might be promoted to most importance
	//			uint16_t r; //Little endiann least significat byte
	//			uint16_t g;
	//			uint16_t b;
	//			uint16_t a; //Most significant byte 
	//		};
	//	};


	//	ColourRGBA16() : value(0) {}
	//};



	//template<typename T>
	//struct ColourRGBA
	//{
	//	static_assert(std::is_integral<T>::value, "T must be an integral type.");
	//	static_assert(sizeof(T) == 1 || sizeof(T) == 2, "T must be 8-bit or 16-bit.");

	//	union {
	//		uint64_t value;
	//		struct {
	//			//depending on development r might be promoted to most importance
	//			T r; //Little endiann least significat byte
	//			T g;
	//			T b;
	//			T a; //Most significant byte 
	//		};
	//		typename std::conditional<sizeof(T) == 1, uint32_t, uint64_t>::type value;
	//	};
	//};

	//using ColourRGBA16 = ColourRGBA<uint16_t>;
	//using ColourRGBA8 = ColourRGBA<uint8_t>;
} //namespace VPHX

