#pragma once
#include "SampleFramework/SampleFramework.h"

enum class TextureWrap : uint8
{
	Repeat, 
	ClampToEdge,
	ClampToBorder,
	MirroredRepeat
};

enum class TextureFilter : uint8
{
	Nearest, 
	Linear
};

enum class FramebufferTarget : uint8_t
{
	Default,
	Draw,
	Read
};

enum class TextureFormat : uint8
{
	R8,
	R16,
	R16F,
	R32F,

	RG8,
	RG16,
	RG16F,

	RGB8,
	RGB16,

	RGBA8,
	SRGBA8,
	RGBA16,
	RGBA16F,
	RGBA32F,

	Depth,
	Depth24,
	Depth32F,
	Depth24Stencil8,

	Default, //Let loader pixk channels, default UBtye
	DefaultByte,  // Let loader pixk channels, force UBtye
	DefaultShort, // Let loader pixk channels, force UBtye
	DefaultFloat, // Let loader pixk channels, force UBtye
};


/// note Default is ised as backend support
/// default = 0, help force backend like stb_img
/// so desired channel count 0, then system detects 
/// img file data channels out. 
enum class PixelFormat : uint8
{
	Default,		//0
	Red,			//1 channel
	RG,				//2 
	RGB,			//3
	RGBA,			//4
	Depth,
	DepthStencil,
};

enum class PixelType : uint8
{
	UByte,
	Float,
	UShort
};

enum class EAttachmentBufferType : uint8_t
{
	Colour,
	Depth,
	DepthStencil,
};



static inline PixelFormat GetPixelFormat(TextureFormat fmt)
{
	switch (fmt)
	{
	case TextureFormat::R8:
	case TextureFormat::R16:
	case TextureFormat::R32F:
	case TextureFormat::R16F: return PixelFormat::Red;

	case TextureFormat::RG8:
	case TextureFormat::RG16:
	case TextureFormat::RG16F: return PixelFormat::RG;

	case TextureFormat::RGB8:
	case TextureFormat::RGB16: return PixelFormat::RGB;

	case TextureFormat::RGBA8:
	case TextureFormat::SRGBA8:
	case TextureFormat::RGBA16:
	case TextureFormat::RGBA16F:
	case TextureFormat::RGBA32F: return PixelFormat::RGBA;

	case TextureFormat::Depth24:
	case TextureFormat::Depth32F: return PixelFormat::Depth;

	case TextureFormat::Depth24Stencil8: return PixelFormat::DepthStencil;

	default: return PixelFormat::Default;
	}
}

static inline int PixelFormatChannelCount(PixelFormat px_format)
{
	switch (px_format)
	{
	case PixelFormat::Default:	return 0; //by default 4 channels
	case PixelFormat::Red:		return 1;
	case PixelFormat::RG:		return 2;
	case PixelFormat::RGB:		return 3;
	case PixelFormat::RGBA:		return 4;
	default:					return 0;
	}
}


/// Pixel Type Size
/// use case 
/// int channels = PixelFormatChannelCount(pxFormat); 1, 2, 3, 4
/// size_t stride = (size_t)channels * PixelTypeSize(pxType);
static inline size_t PixelTypeSize(PixelType type)
{
	switch (type)
	{
	case PixelType::UByte: return sizeof(uint8_t);
	case PixelType::Float: return sizeof(float);
	case PixelType::UShort: return sizeof(uint16_t);
	default: return 0;
	}
}

static inline PixelType PixelTypeFromFormat(TextureFormat fmt)
{
	switch (fmt)
	{
	case TextureFormat::R8:
	case TextureFormat::RG8:
	case TextureFormat::RGB8: 
	case TextureFormat::RGBA8:
	case TextureFormat::SRGBA8:
	case TextureFormat::Default:
	case TextureFormat::DefaultByte: return PixelType::UByte;

	case TextureFormat::R16:
	case TextureFormat::RG16:
	case TextureFormat::RGB16: 
	case TextureFormat::RGBA16:
	case TextureFormat::DefaultShort: return PixelType::UShort;


	case TextureFormat::R16F:
	case TextureFormat::RG16F:
	case TextureFormat::RGBA16F:
	case TextureFormat::R32F:
	case TextureFormat::RGBA32F:
	case TextureFormat::DefaultFloat: return PixelType::Float;



	default: return PixelType::UByte;
	}
}

constexpr bool IsDefault(TextureFormat fmt)
{
	return fmt == TextureFormat::Default || fmt == TextureFormat::DefaultByte ||
		fmt == TextureFormat::DefaultFloat || fmt == TextureFormat::DefaultShort;
}

constexpr bool IsDefault(PixelFormat px_fmt) { return px_fmt == PixelFormat::Default; }

constexpr bool IsDepthFormat(TextureFormat fmt)
{
	return fmt == TextureFormat::Depth24 || 
		fmt == TextureFormat::Depth32F ||
		fmt == TextureFormat::Depth24Stencil8;
}

constexpr bool IsDepthStencilFormat(TextureFormat fmt)
{
	return fmt == TextureFormat::Depth24Stencil8;
}

constexpr EAttachmentBufferType AttachmentTypeFromFormat(TextureFormat fmt)
{
	if (IsDepthStencilFormat(fmt)) return EAttachmentBufferType::DepthStencil;
	if (IsDepthFormat(fmt)) return EAttachmentBufferType::Depth;

	return EAttachmentBufferType::Colour;
}

constexpr TextureFormat DeduceTextureFormat(PixelFormat px_format, PixelType px_type)
{
	struct FormatMapEntry
	{
		PixelFormat px;
		PixelType type;
		TextureFormat fmt;
	};

	constexpr FormatMapEntry default_formats[] = {
		{PixelFormat::Red, PixelType::UByte, TextureFormat::R8},
		{PixelFormat::Red, PixelType::UShort, TextureFormat::R16},
		{PixelFormat::Red, PixelType::Float, TextureFormat::R16F},

		{PixelFormat::RG, PixelType::UByte, TextureFormat::RG8},
		{PixelFormat::RG, PixelType::UShort, TextureFormat::RG16},
		{PixelFormat::RG, PixelType::Float, TextureFormat::RG16F},

		{PixelFormat::RGB, PixelType::UByte, TextureFormat::RGB8},
		{PixelFormat::RGB, PixelType::UShort, TextureFormat::RGB16},

		{PixelFormat::RGBA, PixelType::UByte, TextureFormat::RGBA8},
		{PixelFormat::RGBA, PixelType::UShort, TextureFormat::RGBA16},
		{PixelFormat::RGBA, PixelType::Float, TextureFormat::RGBA16F},
	};


	for (auto& [px, type, fmt] : default_formats)
	{
		if (px == px_format && type == px_type)
			return fmt;
	}

	VX_LOG_WARN("Unsupported pixel format, type match, defaulting to TextureFormat::RGBA8");
	return TextureFormat::RGBA8;
}


#include "ErrorAssertion.h"


namespace GLConvert
{
	static GLint WrapMode(TextureWrap wrap)
	{
		switch (wrap)
		{
		case TextureWrap::Repeat: return GL_REPEAT;
		case TextureWrap::ClampToEdge: return GL_CLAMP_TO_EDGE;
		case TextureWrap::ClampToBorder: return GL_CLAMP_TO_BORDER;
		case TextureWrap::MirroredRepeat: return GL_MIRRORED_REPEAT;
		}

		VX_LOG_WARN("Unsupported wrap, defaulting to GL_REPEAT");
		return GL_REPEAT;
	}

	static GLint FilterMode(TextureFilter filter)
	{
		switch (filter)
		{
		case TextureFilter::Nearest: return GL_NEAREST;
		case TextureFilter::Linear: return GL_LINEAR;
		}

		VX_LOG_WARN("Unsupported filter, defaulting to GL_LINEAR");
		return GL_LINEAR;
	}


	static GLint Format(TextureFormat fmt)
	{
		switch (fmt)
		{
		case TextureFormat::R8:	return GL_R8;
		case TextureFormat::R16: return GL_R16;
		case TextureFormat::R16F: return GL_R16F;
		case TextureFormat::R32F: return GL_R32F;

		case TextureFormat::RG8:	return GL_RG8;
		case TextureFormat::RG16: return GL_RG16;
		case TextureFormat::RG16F: return GL_RG16F;

		case TextureFormat::RGB8: return GL_RGB8;
		case TextureFormat::SRGBA8 : return GL_SRGB8_ALPHA8;
		case TextureFormat::RGB16: return GL_RGB16;

		case TextureFormat::RGBA8: return GL_RGBA8;
		case TextureFormat::RGBA16: return GL_RGBA16;
		case TextureFormat::RGBA16F: return GL_RGBA16F;
		case TextureFormat::RGBA32F: return GL_RGBA32F;

		case TextureFormat::Depth: return GL_DEPTH_COMPONENT;
		case TextureFormat::Depth24: return GL_DEPTH_COMPONENT24;
		case TextureFormat::Depth32F: return GL_DEPTH_COMPONENT32F;
		case TextureFormat::Depth24Stencil8: return GL_DEPTH24_STENCIL8;
		}


		VX_LOG_WARN("Unsupported format, defaulting to GL_RGBA");
		return GL_RGBA;
	}

	static GLint PxFormat(PixelFormat fmt)
	{
		switch (fmt)
		{
		case PixelFormat::Red: return GL_RED;
		case PixelFormat::RG: return GL_RG;
		case PixelFormat::RGB: return GL_RGB;
		case PixelFormat::RGBA: return GL_RGBA;
		case PixelFormat::Depth: return GL_DEPTH_COMPONENT;
		case PixelFormat::DepthStencil: return GL_DEPTH_STENCIL;
		}

		VX_LOG_WARN("Unsupported pixel format, defaulting to GL_RGBA");
		return GL_RGBA;
	}

	static GLint PxType(PixelType type)
	{
		switch (type)
		{
		case PixelType::UByte: return GL_UNSIGNED_BYTE;
		case PixelType::Float: return GL_FLOAT;
		case PixelType::UShort: return GL_UNSIGNED_SHORT;
		}

		VX_LOG_WARN("Unsupported pixel type, defaulting to GL_UNSIGNED_BYTE");
		return GL_UNSIGNED_BYTE;
	}



	static GLenum FBOTarget(FramebufferTarget target)
	{
		switch (target)
		{
		case FramebufferTarget::Default: return GL_FRAMEBUFFER;
		case FramebufferTarget::Draw: return GL_DRAW_FRAMEBUFFER;
		case FramebufferTarget::Read: return GL_READ_FRAMEBUFFER;
		}

		VX_LOG_WARN("Unsupported frame buffer target, defaulting to GL_FRAMEBUFFER");
		return GL_FRAMEBUFFER;
	}

	static GLenum Attachment(EAttachmentBufferType type)
	{
		switch (type)
		{
		case EAttachmentBufferType::Colour: return GL_COLOR_ATTACHMENT0;
		case EAttachmentBufferType::Depth: return GL_DEPTH_ATTACHMENT;
		case EAttachmentBufferType::DepthStencil: return GL_DEPTH_STENCIL_ATTACHMENT;
		}

		VX_LOG_WARN("Unsupported attachment type, defaulting to GL_COLOR_ATTACHMENT0");
		return GL_COLOR_ATTACHMENT0;
	}
}

struct TextureCreateInfo
{
	TextureCreateInfo(uint32 _width = 0, uint32 _height = 0,
					 TextureFormat internal_format = TextureFormat::RGBA8,
					 PixelFormat px_format = PixelFormat::RGBA,
					PixelType px_type = PixelType::UByte, bool mipmap = false, 
					const std::string& _name = {}) :
		width(_width), height(_height),
		internalFormat(internal_format), 
		pxFormat(px_format), pxType(px_type),
		bGenerateMipmaps(mipmap), 
		name(_name)
	{}

	uint32 width = 0;
	uint32 height = 0;
	TextureFormat internalFormat = TextureFormat::RGBA8;
	PixelFormat pxFormat = PixelFormat::RGBA;
	PixelType pxType = PixelType::UByte;

	bool bGenerateMipmaps = false;
	std::string name = {};
	bool isBindless = false;


	static TextureCreateInfo RenderTargetDefault(uint32 w, uint32 h,
		TextureFormat internal_format = TextureFormat::RGBA8,
		PixelFormat px_format = PixelFormat::RGBA,
		PixelType px_type = PixelType::Float, bool mipmap = false)
	{
		return { w, h, internal_format, px_format, px_type, mipmap };
	}
};


struct SamplerCreateInfo
{
	SamplerCreateInfo(TextureWrap s_wrap = TextureWrap::Repeat, TextureWrap t_wrap = TextureWrap::Repeat,
		TextureFilter min_filter = TextureFilter::Linear, TextureFilter mag_filter = TextureFilter::Linear,
		const float border_col[4] = nullptr) :
		wrapS(s_wrap), wrapT(t_wrap), minFilter(min_filter), magFilter(mag_filter)
	{
		if (border_col)
		{
			borderColour[0] = border_col[0];
			borderColour[1] = border_col[1];
			borderColour[2] = border_col[2];
			borderColour[3] = border_col[3];
		}
		else
			borderColour[0] = borderColour[1] = borderColour[2] = borderColour[3] = 1.0f;
	}

	SamplerCreateInfo(TextureWrap wrap, TextureFilter filter, const float border_col[4] = nullptr) : 
		wrapS(wrap), wrapT(wrap), minFilter(filter), magFilter(filter)
	{
		if (border_col)
		{
			borderColour[0] = border_col[0];
			borderColour[1] = border_col[1];
			borderColour[2] = border_col[2];
			borderColour[3] = border_col[3];
		}
		else
			borderColour[0] = borderColour[1] = borderColour[2] = borderColour[3] = 1.0f;
	}

	TextureWrap wrapS = TextureWrap::Repeat;
	TextureWrap wrapT = TextureWrap::Repeat;

	TextureFilter minFilter = TextureFilter::Linear;
	TextureFilter magFilter = TextureFilter::Linear;

	float borderColour[4] = { 1, 1,1, 1 };


	//static SamplerCreateInfo Wrap(TextureWrap wrap,
	//	TextureFilter filter = TextureFilter::Linear,
	//	const float border_col[4] = nullptr)
	//{
	//	return SamplerCreateInfo(wrap, wrap,
	//		filter, filter, border_col);
	//}
};

