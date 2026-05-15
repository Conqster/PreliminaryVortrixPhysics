#include "Font.h"

#define STB_TRUETYPE_IMPLEMENTATION
#include "external/stb_truetype.h"

#include "Shader.h"
#include <array>

///for my own sanity 
using namespace vx;

void Font::Create(const char* font_path, const vx::Ref<Shader> shader)
{

	std::vector<unsigned char> font_buff;
	if (!LoadFontFile(font_buff, font_path))
		return;

	stbtt_fontinfo font;
	if (!stbtt_InitFont(&font, font_buff.data(), 0))
		return;

	float scale = stbtt_ScaleForPixelHeight(&font, float(mCharHeight));
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	int ascent, descent, line_gap;
	stbtt_GetFontVMetrics(&font, &ascent, &descent, &line_gap);

	mTextHeight = (ascent - descent) * scale;

	int atlas_width = mAtlas.kAtlasWidth;
	int atlas_height = mAtlas.kAtlasHeight;

	unsigned char* atlas = new unsigned char[atlas_width * atlas_height];
	memset(atlas, 0, atlas_width * atlas_height);


	//std::vector<unsigned char> atlas[atlas_width * atlas_height];

	int x = 0, y = 0, row_height = 0;

	for (int c = 32; c < kCharCount; ++c)
	{
		int w, h, xoff, yoff;
		unsigned char* bitmap = stbtt_GetCodepointBitmap(&font, 0, scale, c, &w, &h, &xoff, &yoff);
		

		if (x + w + mCharPadding >= atlas_width)
		{
			x = 0;
			y += row_height + mCharPadding;
			row_height = 0;
		}

		for (int j = 0; j < h; ++j)
		{
			for (int i = 0; i < w; ++i)
				atlas[(x + i) + (y + j) * atlas_width] = bitmap[i + j * w];
		}

		vx::Vec2 uv0 = vx::Vec2(float(x) / float(atlas_width), float(y) / float(atlas_height));
		vx::Vec2 uv1 = vx::Vec2(float(x + w) / float(atlas_width), float(y + h) / float(atlas_height));

		//int kern = stbtt_GetCodepointKernAdvance(&font, prevChar)
		x += w + mCharPadding;
		row_height = vx::VxMax(row_height, h);


		int advance, lsb;
		stbtt_GetCodepointHMetrics(&font, c, &advance, &lsb);

		mChars[c] =
		{
			w, h,
			xoff, yoff,
			(int)(advance * scale),

			uv0, uv1
		};

		stbtt_FreeBitmap(bitmap, nullptr);
	}


	//Kern spacing between characters
	for(int i = 32; i < kCharCount; ++i)
		for(int j = 32; j < kCharCount; ++j)
			mKernTable[i][j] = stbtt_GetCodepointKernAdvance(&font, i, j) * scale;

	mAtlas.Generate(atlas);


	CreateVertexBuffer();

	if (shader)
		mShader = shader;
}

void Font::DrawText3D(const std::string_view& text, const vx::Vec3& pos, 
	const vx::Vec3& rt, const vx::Vec3& up, float scale, const vx::Colour& col, ETextAlignment text_alignment)
{

	if (text.empty() || !mShader) return;

	using Create_Func = void(Font::*)(const std::string_view& , const vx::Vec3& ,
		const vx::Vec3&, const vx::Vec3&, float, const vx::Colour&);

	static const Create_Func create_text_table[3] =
	{
		&Font::CreateString<ETextAlignment::Left>,
		&Font::CreateString<ETextAlignment::Center>,
		&Font::CreateString<ETextAlignment::Right>
	};

	(this->*create_text_table[int(text_alignment)])(text, pos, rt, up, scale, col);
}

bool Font::LoadFontFile(std::vector<unsigned char>& buff, const char* path)
{
	std::ifstream file(path, std::ios::binary);
	if (!file)
	{
		std::cout << "Failed to openl font file\n";
		return false;
	}

	buff = std::vector<unsigned char>(
		(std::istreambuf_iterator<char>(file)),
		std::istreambuf_iterator<char>());
	return true;
}

void Font::CreateVertexBuffer()
{
	mVerticesBuffer.Generate(BufferUsage::Vertex, sizeof(TextVertex), kMaxTextVertexCount, nullptr, BufferMemory::DeviceLocal, BufferFrequency::Dynamic);

	std::array<VertexAttribute, 3> attributes =
	{ {
		{1, 2, VertexAttribute::Type::Float, false, offsetof(TextVertex, uv)},
		{0, 3, VertexAttribute::Type::Float, false, offsetof(TextVertex, position)},
		{2, 4, VertexAttribute::Type::UInt8, true, offsetof(TextVertex, colour)},
	} };

	GLCall(glGenVertexArrays(1, &VAO));
	mVerticesBuffer.BindVertexLayout(attributes.data(), attributes.size(), VAO, sizeof(TextVertex));
	mVerticesBuffer.AssignGPULabel("Font Primitive");
}

void Font::PushVertex(const vx::Vec3& pos, float u, float v, const vx::Colour col)
{
	TextVertex vertex;
	pos.Store(vertex.position);
	vertex.uv = { u,v };
	vertex.colour = col;
	mVertices.push_back(vertex);
}

void Font::DrawFrame()
{
	//mVerticesGPU.SegmentUpload(mVertices.data(), mFrameVertexCount * sizeof(TextVertex), mFrameFirstVertex);


	if (mVertices.empty())return;

	size_t byte_size = mVertices.size() * sizeof(TextVertex);

	size_t bytes_offset = mVerticesBuffer.PushData(mVertices.data(), byte_size);
	uint32_t first_vertex = bytes_offset / sizeof(TextVertex);

	mShader->Bind();
	mAtlas.Bind(0);

	GLCall(glBindVertexArray(VAO));
	GLCall(glDrawArrays(GL_TRIANGLES, first_vertex, mVertices.size()));
	GLCall(glBindVertexArray(0));

	Shader::UnbindProgram();

	mVertices.clear();
}

template<ETextAlignment TextAlign>
void Font::CreateString(const std::string_view& text, const vx::Vec3& pos, 
	const vx::Vec3& rt, const vx::Vec3& up, float scale, const vx::Colour& col)
{
	float x = 0;
	if constexpr (TextAlign != ETextAlignment::Left)
	{
		float total_width = 0;
		for (char c : text) total_width += (mChars[c].advance * scale);

		float x_offset = 0.0f;
		if constexpr (TextAlign == ETextAlignment::Center) x_offset = -total_width * 0.5f;
		else x_offset = -total_width;

		x = x_offset;
	}

	//vx::Vec3 pos = i_pos + up * mTextHeight;
	vx::Vec3 base_pos = pos - up * (mTextHeight * scale * 0.5f);
	char prev = 0;
	for (char c : text)
	{
		if (c < 32 || c >= kCharCount)continue;

		const Character& ch = mChars[(int)c];

		float xpos = x + (ch.bearingX * scale);

		//float ypos = -ch.bearingY * scale;
		float ypos = ch.bearingY * scale;



		float w = ch.width * scale;
		float h = ch.height * scale;

		//Build quad in 3D (billboard using rt/up vectors)
		//vx::Vec3 p0 = pos + rt * xpos + up * ypos;
		vx::Vec3 p0 = base_pos + rt * xpos + up * (ypos);
		vx::Vec3 p1 = base_pos + rt * (xpos + w) + up * ypos;
		vx::Vec3 p2 = base_pos + rt * (xpos + w) + up * (ypos + h);
		vx::Vec3 p3 = base_pos + rt * xpos + up * (ypos + h);

		float y_top = (-ch.bearingY * scale);
		float y_bottom = y_top - h;

		//y_top -= (mTextHeight * scale * 0.5f);
		//y_bottom -= (mTextHeight * scale * 0.5f);

		p0 = base_pos + rt * xpos + up * y_bottom; //bottom left
		p1 = base_pos + rt * (xpos + w) + up * y_bottom; //bottom right
		p2 = base_pos + rt * (xpos + w) + up * y_top; //tope right
		p3 = base_pos + rt * xpos + up * y_top; //top left


		//texture coords
		float u0 = ch.UV0.X(), v0 = ch.UV0.Y();
		float u1 = ch.UV1.X(), v1 = ch.UV1.Y();

		PushVertex(p0, u0, v1, col); //bottom lt
		PushVertex(p1, u1, v1, col); //bt rt
		PushVertex(p2, u1, v0, col);//top right

		PushVertex(p0, u0, v1, col); //bottom left
		PushVertex(p2, u1, v0, col);//top rigth
		PushVertex(p3, u0, v0, col);//top left

		int kern = (prev) ? mKernTable[prev][c] : 0.0f;

		x += (ch.advance + (kern)) *scale;
		prev = c;
	}
}

void Font::Atlas::Generate(unsigned char* data)
{
	glGenTextures(1, &texId);
	glBindTexture(GL_TEXTURE_2D, texId);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, kAtlasWidth, kAtlasHeight, 0, GL_RED, GL_UNSIGNED_BYTE, data);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glObjectLabel(GL_TEXTURE, texId, 19, "Font Atlas Texture");
}

void Font::Atlas::Bind(unsigned int slot) const
{
	GLCall(glBindTextureUnit(slot, texId));
}
