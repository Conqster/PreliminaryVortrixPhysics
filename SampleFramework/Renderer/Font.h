#pragma once



#include <vector>

#include <string>
#include <fstream>
#include <sstream>

#include "ErrorAssertion.h"

#include <string>
#include "Vortrix/Maths/Mat44.h"
#include "Vortrix/Maths/Vec3.h"
#include "Vortrix/Core/Colours.h"


#include "GraphicsBuffer.h"

#include "Vortrix/Visuals/ETextAlignment.h"

/// Do not support static 
/// has its goinmf to be used for physics metric 
/// which means values changes freq'ly


class Shader;

class Font
{
public:
	Font() = default;

	void Create(const char* font_path, const vx::Ref<Shader> shader);


	void DrawText3D(const std::string_view& text, const vx::Vec3& pos,
		const vx::Vec3& rt, const vx::Vec3& up,
		float scale, const vx::Colour& col, ETextAlignment text_alignment);


	void DrawFrame();

	~Font() = default;
private:
	static constexpr int kCharCount = 128;

	int mCharHeight = 48;
	float mTextHeight = 1.0f;
	int mCharPadding = 2;

	bool LoadFontFile(std::vector<unsigned char>& buff, const char* path);

	void CreateVertexBuffer();

	template<ETextAlignment TextAlign>
	void CreateString(const std::string_view& text, const vx::Vec3& pos, const vx::Vec3& rt,
						const vx::Vec3& up, float scale, const vx::Colour& col);

	void PushVertex(const vx::Vec3& pos, float u, float v, const vx::Colour col);


	/// Hopefully 128 * 128 * 2bytes = 32KB
	/// into an entire CPU's L1 cache
	std::int16_t mKernTable[kCharCount][kCharCount];

	struct Character
	{
		int width, height;
		int bearingX, bearingY;
		int advance;

		vx::Vec2 UV0;
		vx::Vec2 UV1;
	};
	Character mChars[kCharCount];

	struct Atlas
	{
		unsigned int texId;
		static constexpr int kAtlasWidth = 512;
		static constexpr int kAtlasHeight = 512;

		void Generate(unsigned char* data);
		void Bind(unsigned int slot) const;
	};
	Atlas mAtlas;


	//#pragma pack(push, 1) //GPU oprimisatriton expect struct to be 24 bytes no padding
	struct TextVertex
	{
		vx::Vec2 uv;// enforced 8
		vx::Float3 position; //
		vx::Colour colour; //
	};
	//#pragma pack(pop)
	static_assert(sizeof(TextVertex) == 24, "TextVertex must be exactly 24 bytes");
	/// max text vertex count : 43690 i.e 43690 * size of TextVertex(24) = 1MB
	static constexpr size_t kMaxTextVertexCount = 43690;


	std::vector<TextVertex> mVertices;

	unsigned int VAO = 0;
	RingBuffer mVerticesBuffer;
	vx::Ref<Shader> mShader = nullptr;
};


