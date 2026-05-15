#pragma once

#include "SampleFramework/SampleFramework.h"
#include "ErrorAssertion.h"
#include "TextureParemeter.h"
#include "Texture.h"

#include <functional>


struct TextureRTDesc
{
	TextureCreateInfo ci;
};

struct RenderbufferRTDesc
{
	uint32 width = 0;
	uint32 height = 0;
	TextureFormat format = TextureFormat::Depth24Stencil8;
};

class Renderbuffer
{
public:
	Renderbuffer() = default;
	~Renderbuffer() = default;

	bool CreateRenderTarget(uint32_t w, uint32_t h, TextureFormat internal_format = TextureFormat::Depth24Stencil8)
	{
		mWidth = w;
		mHeight = h;
		mInternalFormat = internal_format;

		glGenRenderbuffers(1, &mID);
		glBindRenderbuffer(GL_RENDERBUFFER, mID);
		glRenderbufferStorage(GL_RENDERBUFFER, GLConvert::Format(internal_format), mWidth, mHeight);

		return true;
	}

	uint32 ID() const { return mID; }
	uint32 Width() const { return mWidth; }
	uint32 Height() const { return mHeight; }
	TextureFormat InternalFormat() const { return mInternalFormat; }

	void Resize(uint32_t width, uint32_t height)
	{
		mWidth = width;
		mHeight = height;
		glBindRenderbuffer(GL_RENDERBUFFER, mID);
		glRenderbufferStorage(GL_RENDERBUFFER, GLConvert::Format(mInternalFormat), mWidth, mHeight);
	}
	void Destroy()
	{
		glDeleteRenderbuffers(1, &mID);
	}
private:
	unsigned int mID = 0;
	uint32 mWidth = 0;
	uint32 mHeight = 0;
	TextureFormat mInternalFormat = TextureFormat::Depth24Stencil8;
};


enum class EAttachmentType : vx::uint8
{
	None,
	Texture,
	Renderbuffer
};

struct IAttachment
{
	//using BufferType = void*;

	IAttachment() = default;
	IAttachment(EAttachmentType _type) : type(_type){}

	virtual ~IAttachment() = default;
	virtual void Resize(uint32_t w, uint32_t h) = 0;
	virtual uint32 ID() const = 0;
	virtual uint32 Width() const = 0;
	virtual uint32 Height() const = 0;
	virtual TextureFormat Format() const = 0;
	virtual void Bind(unsigned int slot = 0) = 0;
	virtual EAttachmentBufferType BufferType() const = 0;

	virtual const void* GetBuffer() const = 0;

	EAttachmentType Type() const { return type; }

protected:
	EAttachmentType type = EAttachmentType::None;
};

struct TextureAttachment : public IAttachment
{
	TextureAttachment(Texture* texture) : mTexture(texture), IAttachment(EAttachmentType::Texture){}
	TextureAttachment() : IAttachment(EAttachmentType::Texture) {}
	~TextureAttachment() = default;

	Texture* mTexture = nullptr;

	void Resize(uint32_t w, uint32_t h) override
	{
		
	}

	uint32 ID() const override { return mTexture->ID(); }
	uint32 Width() const override { return mTexture->Width(); }
	uint32 Height() const override { return mTexture->Height(); }
	TextureFormat Format() const { return mTexture->InternalFormat(); }
	void Bind(unsigned int slot = 0) override { mTexture->Bind(slot); }
	EAttachmentBufferType BufferType() const { return AttachmentTypeFromFormat(mTexture->InternalFormat()); }
	void Set(Texture* tex) { mTexture = tex; }

	const void* GetBuffer() const { return mTexture; }
};


struct RenderbufferAttachment : public IAttachment
{
	RenderbufferAttachment(Renderbuffer* render_buff) : mRenderbuffer(render_buff), IAttachment(EAttachmentType::Renderbuffer){}
	RenderbufferAttachment() : IAttachment(EAttachmentType::Renderbuffer) {}
	~RenderbufferAttachment() = default;

	Renderbuffer* mRenderbuffer = nullptr;

	void Resize(uint32_t w, uint32_t h) override
	{

	}

	uint32 ID() const override { return mRenderbuffer->ID(); }
	uint32 Width() const override { return mRenderbuffer->Width(); }
	uint32 Height() const override { return mRenderbuffer->Height(); }
	TextureFormat Format() const { return mRenderbuffer->InternalFormat(); }
	void Bind(unsigned int slot = 0) override { VX_LOG_DEBUG("Render buffer attachment, doesnt support bind."); }
	EAttachmentBufferType BufferType() const { return AttachmentTypeFromFormat(mRenderbuffer->InternalFormat()); }
	void Set(Renderbuffer* buffer) { mRenderbuffer = buffer; }

	const void* GetBuffer() const { return mRenderbuffer; }
};

struct FrameBlitInfo
{
	class Framebuffer* readFBO = nullptr;
	Framebuffer* writeFBO = nullptr; //if null, means writing into default

	//unsigned int readFBO = 0; 
	//unsigned int writeFBO = 0;
	vx::Vec2 srcCoord{ 0.0f, 0.0f };
	vx::Vec2 srcSize{ 0.0f, 0.0f };

	vx::Vec2 dstCoord{ 0.0f, 0.0f };
	vx::Vec2 dstSize{ 0.0f, 0.0f };

	TextureFilter filter = TextureFilter::Nearest;
	unsigned int mask = GL_DEPTH_BUFFER_BIT;
};


class Framebuffer
{
public:
	Framebuffer() = default;

	void Create()
	{
		glGenFramebuffers(1, &mID);
	}
	//void AttachColour(uint32_t idx, IAttachment* p_attachment);
	void Attach(IAttachment* p_attachment)
	{
		if (p_attachment->BufferType() == EAttachmentBufferType::Colour)
			AttachColour(p_attachment);
		else
			AttachDepth(p_attachment);
	}
	void AttachColour(IAttachment* p_attachment)
	{
		ValidateSize(p_attachment);

		mColourAttachments.push_back(p_attachment);

		uint32_t col_att_idx = mColourAttachments.size() - 1;

		Bind();

		if (p_attachment->Type() == EAttachmentType::Texture)
		{
			glFramebufferTexture2D(GL_FRAMEBUFFER,
				GL_COLOR_ATTACHMENT0 + col_att_idx,
				GL_TEXTURE_2D,
				p_attachment->ID(), 0);
		}
		else if (p_attachment->Type() == EAttachmentType::Renderbuffer)
		{
			glFramebufferRenderbuffer(GL_FRAMEBUFFER,
				GL_COLOR_ATTACHMENT0 + col_att_idx,
				GL_RENDERBUFFER,
				p_attachment->ID());
		}

		VX_ASSERT_WARN(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE,
			"[FRAMEBUFFER ERROR]: Framebuffer did not complete!!!!");
	}
	void AttachDepth(IAttachment* p_attachment)
	{
		ValidateSize(p_attachment);


		VX_ASSERT_WARN(mDepthAttachment == nullptr, "[FRAMEBUFFER WARNING]: already has a depth attachment!!!!");

		mDepthAttachment = p_attachment;

		EAttachmentBufferType buffer_type = AttachmentTypeFromFormat(mDepthAttachment->Format());
		
		Bind();

		if (p_attachment->Type() == EAttachmentType::Texture)
		{
			glFramebufferTexture2D(GL_FRAMEBUFFER,
				GLConvert::Attachment(buffer_type),
				GL_TEXTURE_2D,
				p_attachment->ID(), 0);
		}
		else if (p_attachment->Type() == EAttachmentType::Renderbuffer)
		{
			glFramebufferRenderbuffer(GL_FRAMEBUFFER,
				GLConvert::Attachment(buffer_type),
				GL_RENDERBUFFER,
				p_attachment->ID());
		}


		VX_ASSERT_WARN(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE,
			"[FRAMEBUFFER ERROR]: Framebuffer did not complete!!!!");
	}


	void ValidateSize(IAttachment* p_attachment)
	{
		if (mWidth == 0 && mHeight == 0)
		{
			mWidth = p_attachment->Width();
			mHeight = p_attachment->Height();
		}
		else
		{
			if (p_attachment->Width() != mWidth || p_attachment->Height() != mHeight)
				VX_LOG_WARN("[FRAMEBUFFER]: Attachment size mismatch");
		}
	}
	//resize all buffer
	void Resize(uint32 width, uint32 height)
	{
		//for (uint32 i = 0; i < Count; ++i)
		//{
		//	if (auto& col_att = mColourAttachment[i]; col_att)
		//	{
		//		if (col_att->bAutoResize)
		//			col_att->Resize(width, height);
		//	}
		//}		

		//if (mDepthStencilAttachment && mDepthStencilAttachment->bAutoResize)
		//	mDepthStencilAttachment->Resize(width, height);
	}

	
	void Bind(FramebufferTarget target = FramebufferTarget::Default) const { GLCall(glBindFramebuffer(GLConvert::FBOTarget(target), mID)); }
	unsigned int ID() const { return mID; }



	static void Blit(const FrameBlitInfo& blit_info)
	{
		if (!blit_info.readFBO)
			return;

		blit_info.readFBO->Bind(FramebufferTarget::Read);
		(blit_info.writeFBO) ? blit_info.writeFBO->Bind(FramebufferTarget::Draw) : glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
		glBlitFramebuffer(
			blit_info.srcCoord.X(), blit_info.srcCoord.Y(),
			blit_info.srcSize.X(), blit_info.srcSize.Y(),
			blit_info.dstCoord.X(), blit_info.dstCoord.Y(),
			blit_info.dstSize.X(), blit_info.dstSize.Y(),
			blit_info.mask, GLConvert::FilterMode(blit_info.filter));

		//unsigned int fbo0 = blit_info.readFBO->ID();
		//unsigned int fbo1 = (blit_info.writeFBO) ? blit_info.writeFBO->ID() : 0;
		//glBlitNamedFramebuffer(fbo0, fbo1,
		//	blit_info.srcCoord.X(), blit_info.srcCoord.Y(),
		//	blit_info.srcSize.X(), blit_info.srcSize.Y(),
		//	blit_info.dstCoord.X(), blit_info.dstCoord.Y(),
		//	blit_info.dstSize.X(), blit_info.dstSize.Y(),
		//	blit_info.mask, GLConvert::FilterMode(blit_info.filter));
	}


	void Destroy()
	{
		GLCall(glDeleteFramebuffers(1, &mID));
		mColourAttachments.clear();
		mDepthAttachment = nullptr;
		mID = 0;
	}
private:
	unsigned int mID = 0;
	std::vector<IAttachment*> mColourAttachments;
	IAttachment* mDepthAttachment = nullptr;
	uint32_t mWidth = 0;
	uint32_t mHeight = 0;
};


class IRenderTarget
{
public:
	virtual ~IRenderTarget() = default;

	virtual void Create(uint32_t width, uint32_t height) = 0;
	virtual void Resize(uint32_t width, uint32_t height) = 0;
	virtual void Bind(uint32_t x_offset = 0, uint32_t y_offset = 0) = 0;
	virtual void BindAttachment(uint32_t idx, uint32_t slot) = 0;
	virtual IAttachment* GetAttachment(uint32_t idx) = 0;
	virtual void Destroy() = 0;

	Framebuffer& GetFramebuffer() { return mFramebuffer; }
protected:
	Framebuffer mFramebuffer;
};




class ShadowMapRenderTarget : public IRenderTarget
{
public: 
	ShadowMapRenderTarget() = default;
	~ShadowMapRenderTarget() = default;

	void Create(uint32_t width, uint32_t height) override
	{
		TextureCreateInfo ci;
		ci.width = width;
		ci.height = height;
		ci.internalFormat = TextureFormat::Depth32F;
		ci.pxFormat = PixelFormat::Depth;
		ci.pxType = PixelType::Float;
		ci.name = "ShadowMap";
		mTexture.CreateRenderTarget(ci);
		mDepthAttachment.mTexture = { &mTexture };

		mFramebuffer.Create();
		mFramebuffer.AttachDepth(&mDepthAttachment);
	}

	void Resize(uint32_t width, uint32_t height) override
	{

	}

	void Bind(uint32_t x_offset = 0, uint32_t y_offset = 0) override
	{
		mFramebuffer.Bind();
		uint32_t w = mDepthAttachment.Width();
		uint32_t h = mDepthAttachment.Height();
		GLCall(glViewport(x_offset, y_offset, w, h));
		GLCall(glDrawBuffer(GL_NONE));
		GLCall(glReadBuffer(GL_NONE));
	}

	void BindAttachment(uint32_t idx, uint32_t slot)
	{
		//read shadow map
		//ignore idx; since only one attachment is availabe
		//glActiveTexture(GL_TEXTURE0 + slot);
		//glBindTexture(GL_TEXTURE_2D, mDepthAttachment.ID());
		glBindTextureUnit(slot, mDepthAttachment.ID());
		//mTexture.Bind(slot);
	}

	IAttachment* GetAttachment(uint32_t idx) { return &mDepthAttachment; }

	void Destroy()
	{
		mFramebuffer.Destroy();
		mTexture.Destroy();
	}
private:
	Texture mTexture;
	TextureAttachment mDepthAttachment;

	///
	///TextureAttachment albedo;
	///TextureAttachment normal;
	///TextureAttachment depth;
	///std::array<TextureAttachment*, 3> attachments = {
	/// &albedo, &normal, depth}
	/// 
	/// to support
	/// BindAttachment(2, 0) binds normal 
};


struct RenderTargetDesc
{
	uint32_t width = 0;
	uint32_t height = 0;

	bool bufferAsTexture = true;
	TextureCreateInfo texCI = {};

	bool useDepth = true;
	bool depthAsTexture = false;
	TextureFormat depth = TextureFormat::Depth24Stencil8;

};

#define POLICY_HAS_ATTACHMENT(v) static constexpr bool kHasAttachment = v

template<typename T>
struct AttachmentPolicy;

template<>
struct AttachmentPolicy<TextureRTDesc>
{
	using Buffer = Texture;
	using Attachment = TextureAttachment;
	using Desc = TextureRTDesc;

	POLICY_HAS_ATTACHMENT(true);

	static void Create(Buffer& buff, const Desc& desc)
	{
		buff.CreateRenderTarget(desc.ci);
	}

	static void Resize(Buffer& buff, uint32_t w, uint32_t h) { buff.Resize(w, h); }
};

template<>
struct AttachmentPolicy<RenderbufferRTDesc>
{
	using Buffer = Renderbuffer;
	using Attachment = RenderbufferAttachment;
	using Desc = RenderbufferRTDesc;

	POLICY_HAS_ATTACHMENT(true);

	static void Create(Buffer& buff, const Desc& desc)
	{
		buff.CreateRenderTarget(desc.width, desc.height, desc.format);
	}

	static void Resize(Buffer& buff, uint32_t w, uint32_t h) 
	{
		buff.Resize(w, h);
	}
};

struct NoColour{};
struct NoDepth{};



template<>
struct AttachmentPolicy<NoColour>
{
	using Buffer = NoColour;
	using Attachment = NoColour;
	using Desc = NoColour;

	POLICY_HAS_ATTACHMENT(false);

	static void Create(Buffer& buff, const Desc& desc){}
};

template<>
struct AttachmentPolicy<NoDepth>
{
	using Buffer = NoDepth;
	using Attachment = NoDepth;
	using Desc = NoDepth;

	POLICY_HAS_ATTACHMENT(false);

	static constexpr bool hasDepth = false;

	static void Create(Buffer& buff, const Desc& desc) {}
	static void Resize(Buffer& buff, uint32_t w, uint32_t h) {}
};


template<typename ColourDesc, typename DepthDesc>
class RenderTarget : public IRenderTarget
{
public:
	RenderTarget() = default;
	~RenderTarget() = default;


	void Create(uint32_t width, uint32_t height) override
	{
		//mBuffer.CreateRenderTarget(width, height);
		//mAttachment.Set(&mBuffer);

		//mFramebuffer.Create();
		//mFramebuffer.Attach(&mAttachment);
	}


	void Create(const ColourDesc& colour_desc, const DepthDesc& depth_desc)
	{

		using ColourPolicy = AttachmentPolicy<ColourDesc>;
		using DepthPolicy = AttachmentPolicy<DepthDesc>;

		mFramebuffer.Create();

		if constexpr (ColourPolicy::kHasAttachment)
		{
			ColourPolicy::Create(mColourBuffer, colour_desc);
			mColourAttachment.Set(&mColourBuffer);
			mFramebuffer.Attach(&mColourAttachment);
		}

		if constexpr (DepthPolicy::kHasAttachment)
		{
			DepthPolicy::Create(mDepthBuffer, depth_desc);
			mDepthAttachment.Set(&mDepthBuffer);
			mFramebuffer.Attach(&mDepthAttachment);
		}
	}


	void Resize(uint32_t width, uint32_t height) override
	{
		using ColourPolicy = AttachmentPolicy<ColourDesc>;
		using DepthPolicy = AttachmentPolicy<DepthDesc>;
		if constexpr (ColourPolicy::kHasAttachment)
			ColourPolicy::Resize(mColourBuffer, width, height);

		if constexpr (DepthPolicy::kHasAttachment)
			DepthPolicy::Resize(mDepthBuffer, width, height);
	}

	void Bind(uint32_t x_offset = 0, uint32_t y_offset = 0) override
	{
		uint32_t w, h = 0;
		if constexpr (AttachmentPolicy<ColourDesc>::kHasAttachment)
		{
			w = mColourAttachment.Width();
			h = mColourAttachment.Height();
		}
		else if constexpr (AttachmentPolicy<DepthDesc>::kHasAttachment)
		{
			w = mDepthAttachment.Width();
			h = mDepthAttachment.Height();
		}
		else
			VX_LOG_WARN("No attachment found; undefined viewport");


		mFramebuffer.Bind();
		GLCall(glViewport(x_offset, y_offset, w, h));
		
		if (mBindCallback) mBindCallback(this);
	}

	void BindAttachment(uint32_t idx, uint32_t slot)
	{
		IAttachment* p_att = nullptr;
		switch (idx)
		{
		case 0:
			if constexpr (AttachmentPolicy<ColourDesc>::kHasAttachment)
				p_att = &mColourAttachment;
			break;
		case 1:
			if constexpr (AttachmentPolicy<DepthDesc>::kHasAttachment)
				p_att = &mDepthAttachment;
			break;
		}

		if(p_att == nullptr) VX_LOG_WARN("No attachment found;");
		p_att->Bind(slot);
	}

	IAttachment* GetAttachment(uint32_t idx)
	{
		switch (idx)
		{
		case 0:
			if constexpr (AttachmentPolicy<ColourDesc>::kHasAttachment)
				return &mColourAttachment;
			break;
		case 1:
			if constexpr (AttachmentPolicy<DepthDesc>::kHasAttachment)
				return &mDepthAttachment;
			break;

		default:
			VX_ASSERT_WARN("out of bound attachment idx");
			return nullptr;
		}
	}

	void Destroy()
	{
		mFramebuffer.Destroy();
		if constexpr (AttachmentPolicy<ColourDesc>::kHasAttachment)
			mColourBuffer.Destroy();
		if constexpr (AttachmentPolicy<DepthDesc>::kHasAttachment)
			mDepthBuffer.Destroy();
	}


	using BindCallback = std::function<void(const RenderTarget* target)>;
	void SetBindCallback(const BindCallback& cb) { mBindCallback = cb; }
private:
	typename AttachmentPolicy<ColourDesc>::Buffer mColourBuffer;
	typename AttachmentPolicy<ColourDesc>::Attachment mColourAttachment;

	typename AttachmentPolicy<DepthDesc>::Buffer mDepthBuffer;
	typename AttachmentPolicy<DepthDesc>::Attachment mDepthAttachment;

	BindCallback mBindCallback;
};


template<size_t N>
class MultiRenderTarget
{
	std::array<Texture, N> mTexture;
};