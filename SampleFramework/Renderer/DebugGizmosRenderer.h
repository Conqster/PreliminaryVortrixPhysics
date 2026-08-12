#pragma once

#include "Vortrix/Core/NonCopyable.h"
#include "Shader.h"

#include <SampleFramework/SampleFramework.h>
#include "TextureParemeter.h"


#include <Vortrix/Visuals/Renderers.h>


class UniformBuffer : vx::NonCopyable
{
public:
	UniformBuffer() = default;
	~UniformBuffer() { Destroy(); }

	void Generate(size_t size, const void* data = nullptr, GLenum usage = GL_DYNAMIC_DRAW)
	{
		mSize = size;
		glCreateBuffers(1, &mID);
		glNamedBufferData(mID, (GLsizeiptr)mSize, data, usage);
	}

	//bind whole buffer to a slot
	void Bind(uint32_t block_slot_idx) const
	{
		GLCall(glBindBufferRange(GL_UNIFORM_BUFFER, block_slot_idx, mID, 0, (GLsizeiptr)mSize));
	}

	//binding specifi ranges [must 256-bytes aligned offset]
	void BindRange(uint32_t block_slot_idx, uint32_t offset, size_t size) const
	{
		GLCall(glBindBufferRange(GL_UNIFORM_BUFFER, block_slot_idx, mID, offset, (GLsizeiptr)size));
	}

	//Full update
	void SetBufferData(const void* data) const
	{
		glNamedBufferSubData(mID, 0, (GLsizeiptr)mSize, data);
	}

	//Partial update
	void SetSubData(const void* data, uint32_t offset, size_t size) const
	{
		glNamedBufferSubData(mID, offset, (GLsizeiptr)size, data);
	}

	void Destroy() { GLCall(glDeleteBuffers(1, &mID)); mID = 0; }
	size_t GetSize() const { return mSize; }

private:
	unsigned int mID = 0;
	size_t mSize = 0;
	//GLenum mUsage = GL_DYNAMIC_DRAW
}; //Uniform Buffer class 

class ShaderStorageBuffer : vx::NonCopyable
{
public:
	ShaderStorageBuffer() = default;
	~ShaderStorageBuffer() { Destroy(); }

	void Generate(size_t size, const void* data = nullptr, GLenum usage = GL_DYNAMIC_DRAW)
	{
		mSize = size;
		glCreateBuffers(1, &mID);
		glNamedBufferData(mID, (GLsizeiptr)mSize, data, usage);
	}

	//bind whole buffer to a slot
	void Bind(uint32_t block_slot_idx) const
	{
		GLCall(glBindBufferRange(GL_UNIFORM_BUFFER, block_slot_idx, mID, 0, (GLsizeiptr)mSize));
	}

	//binding specifi ranges [must 256-bytes aligned offset]
	void BindRange(uint32_t block_slot_idx, uint32_t offset, size_t size) const
	{
		GLCall(glBindBufferRange(GL_UNIFORM_BUFFER, block_slot_idx, mID, offset, (GLsizeiptr)size));
	}

	//Full update
	void SetBufferData(const void* data) const
	{
		glNamedBufferSubData(mID, 0, (GLsizeiptr)mSize, data);
	}

	//Partial update
	void SetSubData(const void* data, uint32_t offset, size_t size) const
	{
		glNamedBufferSubData(mID, offset, (GLsizeiptr)size, data);
	}

	void Destroy() { GLCall(glDeleteBuffers(1, &mID)); mID = 0; }
	size_t GetSize() const { return mSize; }

private:
	unsigned int mID = 0;
	size_t mSize = 0;
	//GLenum mUsage = GL_DYNAMIC_DRAW
}; //Uniform Buffer class 


namespace vx {
	struct AABB;
}

struct GizmosCameraData
{
	vx::Mat44 proj;
	vx::Mat44 view;
};

//later do not take in cam
struct DrawCommand
{
	GizmosCameraData cameraData;

	struct BufferRange
	{
		size_t start = 0;
		size_t count = 0;
	};

	BufferRange lineBuffRange;
	BufferRange triBuffRange;

	//if rendertagert null write to defult framebuffer
	class IRenderTarget* renderTarget = nullptr;
};

class DebugGizmosRendererImpl : public vx::DebugGizmosRenderer
{
public:
	DebugGizmosRendererImpl() = default;
	~DebugGizmosRendererImpl() = default;


	bool Init(class ApplicationWindow* window) override;


	void DrawLine(const vx::Vec3& v0, const vx::Vec3& v1, const vx::Colour& colour) override;
	void DrawWireTriangle(const vx::Vec3& v1, const vx::Vec3& v2, const vx::Vec3& v3, const vx::Colour& colour) override;
	void DrawSolidTriangle(const vx::Vec3& v1, const vx::Vec3& v2, const vx::Vec3& v3, const vx::Colour& colour) override;
	void DrawSolidWireTriangle(const vx::Vec3& v1, const vx::Vec3& v2, const vx::Vec3& v3, const vx::Colour& colour) override;
	//void DrawTriangle(vx::Vec3 v0, vx::Vec3 v1, vx::Vec3 v2, glm::vec4 colour, bool cull_face = true);
	void DrawWireSphereDiscs(const vx::Vec3& center, float radius, const vx::Colour col, int segments = 48) override;
	void DrawWireDisc(const vx::Vec3& center, float radius, const vx::Colour col, const vx::Vec3& plane_axis_x = vx::Vec3::Right(),
		const vx::Vec3& plane_axis_y = vx::Vec3::Up(), int segments = 48) override;
	void DrawHalfWireDisc(const vx::Vec3& center, float radius, const vx::Colour col, const vx::Vec3& plane_axis_x = vx::Vec3::Right(),
		const vx::Vec3& plane_axis_y = vx::Vec3::Up(), int segments = 48) override;
	void DrawWireDisc(const vx::Vec3& center, float radius, float ratio, const vx::Colour col, const vx::Vec3& plane_axis_x = vx::Vec3::Right(),
		const vx::Vec3& plane_axis_y = vx::Vec3::Up(), int segments = 48) override;
	void DrawArrow(const vx::Vec3& v0, const vx::Vec3& v1, const vx::Vec3& base_axis, float width, float height, vx::Colour col) override;

	void DrawCone(const vx::Vec3& apex, const vx::Vec3& base_center, float radius, int segments = 12, vx::Colour col = vx::Colour::sMagenta);
	void DrawArrowCone(const vx::Vec3& start, const vx::Vec3& end,
		float shaft_radius, float head_height, float head_radius, 
		int cone_segment = 12, vx::Colour col = vx::Colour::sMagenta) override;
	/// draw a rectangle prism from corners 
	void DrawBox(const std::array<vx::Vec3, 8>& corners, const vx::Colour& col, bool wireframe = true);
	void DrawAABB(const vx::Vec3& min, const vx::Vec3& max, const vx::Colour& col, bool wireframe = true);
	void DrawAABB(const vx::AABB& aabb, const vx::Colour& col, bool wireframe = true) override;

	template<size_t Sector = 8, size_t Stack = 6, bool Wireframe = false>
	void DrawSphere(const vx::Vec3& center, float radius, vx::Colour col);
	//void DrawAABB(const vx::AABB& aabb, const vx::Colour& col, bool wireframe = false);

	void DrawSphere(const vx::Vec3& center, float radius, vx::Colour col) override { DrawSphere<8, 6, false>(center, radius, col); }
	void DrawWireSphere(const vx::Vec3& center, float radius, vx::Colour col) override { DrawSphere<8, 6, true>(center, radius, col); }
	void DrawSphere4x4(const vx::Vec3& center, float radius, vx::Colour col) override { DrawSphere<4, 4, false>(center, radius, col); }

	/// axis aligned cross 
	void DrawAACross(const vx::Vec3& pos, const Colour* colour, int colour_count, float scale = 1.0f) override;
	void DrawCross(const vx::Vec3& pos, const vx::Vec3& rt,
		const vx::Vec3& up, const vx::Vec3& fwd, const Colour* colour, int colour_count, float scale = 1.0f) override;


	void DrawBasis(const vx::Vec3& pos, const vx::Vec3& rt,
		const vx::Vec3& up, const vx::Vec3& fwd,
		float arrow_width, float arrow_height, 
		bool b_draw_plane = true, uint32_t cone_segment = 8) override;

	void DrawBasis(const vx::Mat44 transform, float arrow_width, float arrow_height, bool b_draw_plane = true, uint32_t cone_segment = 8) override;

	
	void UploadIfDirty();
	void ExecuteDraws();

	bool SetShader(Shader* shader) 
	{
		(mShader) ? mShader->Clear() : void(); //<-- important to prevent cache data corruption
		mShader = (shader) ? shader : mShader;
		if (mShader == nullptr)
		{
			mShader = new Shader();
			mShader->Create("debug-shader",
				"assets/shaders/debug/batchLines.vert", //vertex shader
				"assets/shaders/debug/batchLines.frag"); //fragment shader
		}
		return (mShader != nullptr);
	}

	float GetLineWidth() const override { return mLineWidth; }
	void SetLineWidth(float value) override {
		if (!this){
			VX_LOG_WARN("Failed to set debug gizmos renderer line width, memory error.");
			return;
		}
		mLineWidth = value;
	}


	void Destroy(bool flush = false)
	{
		if (flush)
			ExecuteDraws();

		mCameraUBO.Destroy();

		mShader->Clear();

		mLineBatches.clear();
		mTriangleBatches.clear();

		mLineVertexGrp.Destroy();
		mTriVertexGrp.Destroy();

		mDrawCommands.clear();
	}


	struct Line
	{
		//vertex 0
		vx::Float3 from;
		vx::Colour fromColour;

		//vertex 1
		vx::Float3 to;
		vx::Colour toColour;

		float thickness;
	};

	//triangle Vertices 
	//struct Vertex
	//{
	//	vx::Float3 position;
	//	vx::Float3 normal;
	//	vx::Vec2 uv;
	//	vx::Vec4 colour; // vx::Colour colour;
	//};

	struct VertexData
	{
		vx::Float3 position;
		vx::Float3 normal;
		vx::Colour colour;
	};

	struct Triangle
	{
		//float v0[3]{ 0.0f }; float v0Colour[4]{ 0.0f };			// <--- vertex0
		//float v1[3]{0.0f}; float v1Colour[4]{ 0.0f };			// <--- vertex1
		//float v2[3]{0.0f}; float v2Colour[4]{ 0.0f };			// <--- vertex2

		VertexData v0;
		VertexData v1;
		VertexData v2;
	};
private:
	Shader* mShader = nullptr;
	float mLineWidth = 1.0f;
	ApplicationWindow* mActiveWindow = nullptr;
#pragma region HelperStructure


	enum class EVertexPrimitiveMode : uint8_t
	{
		Points, 
		Lines, 
		Triangles,
		TriangleStrip
	};

	struct GPUVertexAttribute
	{
		uint32_t index = 0;
		uint32_t size = 0;
		GLenum type = GL_FLOAT;
		bool normalised = false;
		uint32_t offset = 0;
	};

	struct GPUVertexAttributeDivisor
	{
		uint32_t index = 0;
		uint32_t divisor = 1;
	};

	template<EVertexPrimitiveMode VertexPrimitive>
	struct VertexGroup
	{
		unsigned int VAO = 0;
		unsigned int VBO = 0;

		unsigned int maxVertex = 1000;
		unsigned int stride = 0;
		bool bBufferDirty = false;
		unsigned int attributeDivisorCount = 0;
		vx::Ref<Shader> shader;

		void Generate(GLsizei _stride, Ref<Shader> _shader)
		{
			stride = _stride;
			shader = _shader;

			glCreateVertexArrays(1, &VAO);

			glCreateBuffers(1, &VBO);
			glNamedBufferData(VBO, stride * maxVertex, nullptr, GL_DYNAMIC_DRAW);
		}

		void Generate(GLsizei _stride, uint32_t vertices_count, const void* data, GLenum usage = GL_DYNAMIC_DRAW/*, Ref<Shader> _shader = {}*/)
		{
			stride = _stride;
			//shader = _shader;
			maxVertex = vertices_count;

			glCreateVertexArrays(1, &VAO);

			glCreateBuffers(1, &VBO);
			glNamedBufferData(VBO, stride * maxVertex, data, usage);
		}

		void BindLayout(const GPUVertexAttribute* attrib, int attrib_count, const GPUVertexAttributeDivisor* attrib_divs, int attrib_div_count)
		{
			//also helps to link VAO to VBO
			glBindVertexArray(VAO);
			glBindBuffer(GL_ARRAY_BUFFER, VBO);

			for (const GPUVertexAttribute* att = attrib, *att_end = attrib + attrib_count;
				att < att_end; ++att)
			{
				const auto& a = (*att);
				glEnableVertexAttribArray(a.index);
				glVertexAttribPointer(a.index, a.size,
					a.type, a.normalised, stride,
					(const void*)(uintptr_t)a.offset);
			}

			for(const GPUVertexAttributeDivisor* att_div = attrib_divs, *att_div_end = attrib_divs + attrib_div_count;
				att_div < att_div_end; ++att_div)
				glVertexAttribDivisor((*att_div).index, (*att_div).divisor);

			attributeDivisorCount = attrib_div_count;
		}


		void Bind() { GLCall(glBindVertexArray(VAO)); }



		VX_INLINE GLenum GetGLPrimitiveMode()
		{
			if constexpr (VertexPrimitive == EVertexPrimitiveMode::Lines)
				return GL_POINTS;
			else if constexpr (VertexPrimitive == EVertexPrimitiveMode::Lines)
				return GL_LINES;
			else if constexpr (VertexPrimitive == EVertexPrimitiveMode::Triangles)
				return GL_TRIANGLES;
			else if constexpr (VertexPrimitive == EVertexPrimitiveMode::TriangleStrip)
				return GL_TRIANGLE_STRIP;
		}

		VX_INLINE size_t GetVertexCountDivisor() const { return attributeDivisorCount; }

		void DrawArray(int first, size_t count)
		{
			glDrawArrays(GetGLPrimitiveMode(), first, GLsizei(count));
		}

		void DrawArrayInstancedBase(int first, size_t instance_count, size_t primitive_count)
		{
			//glDrawArraysInstanced(GetGLPrimitiveMode(), first, GLsizei(instance_count), GLsizei(primitive_count));
			glDrawArraysInstancedBaseInstance(GetGLPrimitiveMode(), 0, GLsizei(instance_count), GLsizei(primitive_count), first);
		}

		void Destroy()
		{
			glDeleteBuffers(1, &VBO);
			glDeleteVertexArrays(1, &VAO);
		}

	};

	UniformBuffer mCameraUBO;

#pragma endregion
	//probably have a pointer to this to have multiple debug giamos renderer but share GPU data{vertex & shader program)
	VertexGroup<EVertexPrimitiveMode::TriangleStrip> mLineVertexGrp {};
	std::vector<Line> mLineBatches;
	VertexGroup<EVertexPrimitiveMode::Triangles> mTriVertexGrp {};
	std::vector<Triangle> mTriangleBatches;


	std::vector<DrawCommand> mDrawCommands;
public:
	//using VertexAttri = VertexGroup::Attribute;
	//bool Initialise(Shader* shader = nullptr, const std::vector<VertexAttri>& vert_attribs = {}, bool accumulate_offset = true);
	//bool Initialise(Shader* shader = nullptr, std::string_view proj = {}, std::string_view view = {}, const std::vector<VertexAttri>& vert_attribs = {}, bool accumulate_offset = true);


	DrawCommand* PushDrawCommand(DrawCommand cmd) override;
	DrawCommand* PushDrawCommand(vx::Mat44 proj, vx::Mat44 view, IRenderTarget* render_target) override;

	bool EndCurrentDrawCommand() override;

	//template<template T>
	template<EVertexPrimitiveMode T>
	void Upload(VertexGroup<T>& target_vertex_buff, uint32_t vertices_per_shape, size_t buffer_shape_count, const void* buffer_data);

	vx::Vec3 ComputeTriangleNormals(const Vec3& v0, const Vec3& v1, const Vec3& v2)
	{
		//vx::Vec3 v0(v0[0], tri.v0[1], tri.v0[2]);
		//vx::Vec3 v1(v1[0], tri.v1[1], tri.v1[2]);
		//vx::Vec3 v2(v2[0], tri.v2[1], tri.v2[2]);

		return (v1 - v0).Cross(v2 - v0).Normalise();
	}

	void RemoveDrawCommand(DrawCommand& cmd) override;
	bool ExecuteDraw(IRenderTarget* render_target) override;
	bool ExecuteDraw(const DrawCommand& cmd) override;

	bool Flush(IRenderTarget* render_target) override;

private:

	bool InternalExecuteDraw(const DrawCommand& cmd);
};


extern template void DebugGizmosRendererImpl::DrawSphere<8, 6, false>(const Vec3&, float, vx::Colour);
extern template void DebugGizmosRendererImpl::DrawSphere<8, 6, true>(const Vec3&, float, vx::Colour);
