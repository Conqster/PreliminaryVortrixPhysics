#include <SampleFramework/SampleFramework.h>

#include "Texture.h"
#include <stb_image/stb_image.h>

#include "ErrorAssertion.h"


template<PixelType Type>
void* TextureLoader::LoadData(const char* path, int* o_width, int* o_height, int* o_channel, bool flip, int desired_channels)
{
	stbi_set_flip_vertically_on_load(flip);

	if constexpr(Type == PixelType::UByte)
		return stbi_load(path, o_width, o_height, o_channel, desired_channels);
	else if constexpr(Type == PixelType::Float)
		return stbi_loadf(path, o_width, o_height, o_channel, desired_channels);
	else if constexpr(Type == PixelType::UShort)
		return stbi_load_16(path, o_width, o_height, o_channel, desired_channels);
	else
	{
		VX_ASSERT_WARN(false, "Unknown pixel type; fallback unsigned byte/char!!!");
		return stbi_load(path, o_width, o_height, o_channel, desired_channels);
	}
}
/// explicit instaniation for linker
template void* TextureLoader::LoadData<PixelType::UByte>(const char*, int*, int*, int*, bool, int);
template void* TextureLoader::LoadData<PixelType::Float>(const char*, int*, int*, int*, bool, int);
template void* TextureLoader::LoadData<PixelType::UShort>(const char*, int*, int*, int*, bool, int);


void TextureLoader::FreeBuffer(void* buff)
{
	stbi_image_free(buff);
}


bool Texture::CreateFromPixel(const void* data, const TextureCreateInfo& params)
{
	//TO-DO: need to check if this texture is valid/already created
	

	mWidth = params.width;
	mHeight = params.height;

	mInternalFormat = params.internalFormat;
	bMipmaped = params.bGenerateMipmaps;

	//glGenTextures(1, &mID);
	//glBindTexture(GL_TEXTURE_2D, mID);

	//GLCall(glTexImage2D(GL_TEXTURE_2D, 0,
	//	GLConvert::Format(mInternalFormat),
	//	mWidth, mHeight, 0,
	//	GLConvert::PxFormat(params.pxFormat),
	//	GLConvert::PxType(params.pxType), data));

	glCreateTextures(GL_TEXTURE_2D, 1, &mID);
	glTextureStorage2D(mID, 1, GLConvert::Format(mInternalFormat), mWidth, mHeight);

	if(data)
	{
		glTextureSubImage2D(mID, 0, 0, 0, mWidth, mHeight,
			GLConvert::PxFormat(params.pxFormat),
			GLConvert::PxType(params.pxType), data);
	}


	//GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT));
	//GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT));

	//GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST));
	//GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST));


	if (bMipmaped)
		glGenerateMipmap(GL_TEXTURE_2D);

	GLCall(glBindTexture(GL_TEXTURE_2D, 0));

	if (!params.name.empty())
		SetDebugName(params.name);

	if (params.isBindless)
		GenerateBindless();

	return true;


}

bool Texture::CreateRenderTarget(const TextureCreateInfo& params)
{
	if (IsValid())
		Destroy();

	mWidth = params.width;
	mHeight = params.height;

	mInternalFormat = params.internalFormat;
	bMipmaped = params.bGenerateMipmaps;

	//glGenTextures(1, &mID);
	//glBindTexture(GL_TEXTURE_2D, mID);

	//GLCall(glTexImage2D(GL_TEXTURE_2D, 0,
	//	GLConvert::Format(mInternalFormat),
	//	mWidth, mHeight, 0,
	//	GLConvert::PxFormat(params.pxFormat),
	//	GLConvert::PxType(params.pxType), nullptr));

	glCreateTextures(GL_TEXTURE_2D, 1, &mID);
	glTextureStorage2D(mID, 1, GLConvert::Format(mInternalFormat), mWidth, mHeight);
	//glTextureSubImage2D(mID, 0, 0, 0, mWidth, mHeight,
	//	GLConvert::PxFormat(params.pxFormat),
	//	GLConvert::PxType(params.pxType), nullptr);


	if (bMipmaped)
		glGenerateMipmap(GL_TEXTURE_2D);

	GLCall(glBindTexture(GL_TEXTURE_2D, 0));

	if (params.isBindless)
		GenerateBindless();

	if (!params.name.empty())
		SetDebugName(params.name);

	return true;
}

bool Texture::CreateRenderTargets(Texture textures[], const std::vector<TextureCreateInfo>& params)
{
	uint32 textures_count = params.size();

	//unsigned int gpu_texture_ids[textures_count];
	std::vector<unsigned int> gpu_texture_ids(textures_count, 0);

	//glGenTextures(textures_count, &gpu_texture_ids[0]);
	glGenTextures(textures_count, gpu_texture_ids.data());

	for (uint32 i = 0; i < textures_count; ++i)
	{
		const auto& [w, h, format, px_format, px_type, gen_mipmap, name, is_bindless] = params[i];
		auto& curr_texture = textures[i];

		curr_texture.mID = gpu_texture_ids[i];

		curr_texture.mWidth = w;
		curr_texture.mHeight = h;

		curr_texture.mInternalFormat = format;
		curr_texture.bMipmaped = gen_mipmap;


		glBindTexture(GL_TEXTURE_2D, curr_texture.mID);

		GLCall(glTexImage2D(GL_TEXTURE_2D, 0,
			GLConvert::Format(curr_texture.mInternalFormat),
			curr_texture.mWidth, curr_texture.mHeight, 0,
			GLConvert::PxFormat(px_format),
			GLConvert::PxType(px_type), nullptr));


		if (curr_texture.bMipmaped)
			glGenerateMipmap(GL_TEXTURE_2D);

		if (!name.empty())
			curr_texture.SetDebugName(name);
	}
	

	GLCall(glBindTexture(GL_TEXTURE_2D, 0));
	return true;
}

bool Texture::Resize(uint32_t width, uint32_t height)
{
	if (!IsValid())
	{
		VX_LOG_DEBUG("Cant resize invalid texture");
		return false;
	}

	mWidth = width;
	mHeight = height;

	bool was_bindless = false;
	if (mBindlessHandle != 0)
	{
		glMakeTextureHandleNonResidentARB(mBindlessHandle);
		mBindlessHandle = 0;
		was_bindless = true;
	}

	//glBindTexture(GL_TEXTURE_2D, mID);
	//GLCall(glTexImage2D(GL_TEXTURE_2D, 0,
	//	GLConvert::Format(mInternalFormat),
	//	mWidth, mHeight, 0,
	//	GLConvert::PxFormat(PixelFormat()),
	//	GLConvert::PxType(PixelType()), nullptr));

	glTextureStorage2D(mID, 1, GLConvert::Format(mInternalFormat), mWidth, mHeight);
	GLCall(glTextureSubImage2D(mID, 0,0, 0,
	mWidth, mHeight,
	GLConvert::PxFormat(PixelFormat()),
	GLConvert::PxType(PixelType()), nullptr));


	if (bMipmaped)
		glGenerateMipmap(GL_TEXTURE_2D);

	if (was_bindless)
	{
		mBindlessHandle = glGetTextureHandleARB(mID);
		glMakeTextureHandleNonResidentARB(mBindlessHandle);
	}


	GLCall(glBindTexture(GL_TEXTURE_2D, 0));



	return true;
}


bool Texture::Bind(unsigned int slot) const
{
	//glBindSampler(slot, mSID);
	//GLCall(glActiveTexture(GL_TEXTURE0 + slot));
	//GLCall(glBindTexture(GL_TEXTURE_2D, mID));
	glBindTextureUnit(slot, mID);
	return true;
}

void Texture::Unbind() const
{
	GLCall(glBindTexture(GL_TEXTURE_2D, 0));
}

void Texture::Destroy()
{
	if (IsValid())
	{

		if (mBindlessHandle != 0)
		{
			glMakeTextureHandleNonResidentARB(mBindlessHandle);
			mBindlessHandle = 0;
		}

		GLCall(glDeleteTextures(1, &mID));
		mID = 0;
		mWidth = 0;
		mHeight = 0;
	}
}

void Texture::ApplyGPUDebugName(const std::string& name)
{
	if (!name.empty() && mID != 0)
		glObjectLabel(GL_TEXTURE, mID, name.size(), name.c_str());
}



bool Sampler::Create(const SamplerCreateInfo& sampler_ci)
{
	mWrapModeS = sampler_ci.wrapS;
	mWrapModeT = sampler_ci.wrapT;
	mMinFilter = sampler_ci.minFilter;
	mMagFilter = sampler_ci.magFilter;

	bool clamp_border = mWrapModeS == TextureWrap::ClampToBorder || mWrapModeT == TextureWrap::ClampToBorder;

	if(clamp_border)
	{
		for (uint32 i = 0; i < 4; ++i)
			mBorderColour[i] = sampler_ci.borderColour[i];
	}

	glGenSamplers(1, &mID);
	glSamplerParameteri(mID, GL_TEXTURE_WRAP_S, GLConvert::WrapMode(mWrapModeS));
	glSamplerParameteri(mID, GL_TEXTURE_WRAP_T, GLConvert::WrapMode(mWrapModeT));
	glSamplerParameteri(mID, GL_TEXTURE_MIN_FILTER, GLConvert::FilterMode(mMinFilter));
	glSamplerParameteri(mID, GL_TEXTURE_MAG_FILTER, GLConvert::FilterMode(mMagFilter));

	if(clamp_border)
		glSamplerParameterfv(mID, GL_TEXTURE_BORDER_COLOR, mBorderColour);

	return true;
}

void Sampler::Bind(unsigned int slot)
{
	glBindSampler(slot, mID);
}

void Sampler::Unbind()
{
	
}

void Sampler::Destroy()
{
	glDeleteSamplers(1, &mID);
}
