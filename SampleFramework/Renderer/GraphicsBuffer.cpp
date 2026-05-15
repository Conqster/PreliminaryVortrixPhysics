#include "GraphicsBuffer.h"

#include "Vortrix/Core/Core.h"
#include "Vortrix/Core/Logger.h"
#include "Vortrix/Core/Assertion.h"

#include "Vortrix/Core/StackString.h"


///for my own sanity 
using namespace vx;


void GraphicsBuffer::Generate(BufferUsage _type, size_t _size, const void* data, BufferMemory mem, BufferFrequency usage_freq)
{
	if (_type == BufferUsage::Vertex || _type == BufferUsage::Index)
	{
		VX_LOG_WARN("vertex and index buffer requires stride and count, buffer not created");
		return;
	}

	type = _type;
	size = _size;
	glCreateBuffers(1, &iD);
	glNamedBufferData(iD, size, data, InferGLUsage(mem, usage_freq));
}

void GraphicsBuffer::Generate(BufferUsage _type, GLsizei data_stride, vx::uint32 _count, const void* data, BufferMemory mem, BufferFrequency usage_freq)
{
	size = data_stride * _count; //index/vertx
	count = _count;
	type = _type;

	glCreateBuffers(1, &iD);
	glNamedBufferData(iD, size, data, InferGLUsage(mem, usage_freq));
}

void GraphicsBuffer::BindVertexLayout(const VertexAttribute* attrib, int count, const vx::uint32 VAO, uint32_t stride)
{
	VX_ASSERT_WARN_VOID(type == BufferUsage::Vertex, "Only vertex buffer support Layout binding");

	//also helps to link VAO to VBO
	glBindVertexArray(VAO);
	glBindBuffer(GL_ARRAY_BUFFER, iD);

	//quick hack 
	auto Type_GL = [](VertexAttribute::Type type)
		{
			switch (type)
			{
			case VertexAttribute::Type::Float: return GL_FLOAT;
			case VertexAttribute::Type::Int32: return GL_INT;
			case VertexAttribute::Type::UInt8: return GL_UNSIGNED_BYTE;
			case VertexAttribute::Type::UInt32: return GL_UNSIGNED_INT;
			}

			return GL_FLOAT;
		};



	for (uint32_t i = 0; i < count; ++i)
	{
		const auto& a = attrib[i];
		glEnableVertexAttribArray(a.index);
		glVertexAttribPointer(a.index, a.size,
			Type_GL(a.type), a.normalised, stride,
			(const void*)(uintptr_t)a.offset);
	}
}

//use templated fot instancaiom
void GraphicsBuffer::Bind(vx::uint32 slot_point) const
{
	if (type == BufferUsage::Uniform)
		GLCall(glBindBufferRange(GL_UNIFORM_BUFFER, slot_point, iD, 0, (GLsizeiptr)size));
	else if (type == BufferUsage::Storage)
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, slot_point, iD);

	else if (type == BufferUsage::Vertex)
		glBindBuffer(GL_ARRAY_BUFFER, iD);
	else if (type == BufferUsage::Index)
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, iD);
}

void GraphicsBuffer::SegmentBind(size_t size, vx::uint32 slot_point, uint32_t offset) const
{

	if (type != BufferUsage::Uniform)
	{
		VX_LOG_WARN("Only uniform buffer support Segment binding");
		return;
	}
	GLCall(glBindBufferRange(GL_UNIFORM_BUFFER, slot_point, iD, offset, (GLsizeiptr)size));
}

void GraphicsBuffer::ValidateSize(size_t bytes_needed, const void* data)
{
	if (size < bytes_needed)
	{
		//size = bytes_needed * 2;
		size = bytes_needed; //<- might has allocation behaviour 
		GLCall(glNamedBufferData(iD, size, data, GL_DYNAMIC_DRAW));
	}
}



void GraphicsBuffer::ValidateSize(size_t bytes_needed, const void* data, BufferMemory mem, BufferFrequency usage_freq)
{
	if (size < bytes_needed)
	{
		//size = bytes_needed * 2;
		size = bytes_needed; //<- might has allocation behaviour 
		GLCall(glNamedBufferData(iD, size, data, InferGLUsage(mem, usage_freq)));
	}
}

void GraphicsBuffer::Upload(const void* data, uint32_t offset)
{
	GLCall(glNamedBufferSubData(iD, offset, size, data));
}

void GraphicsBuffer::SegmentUpload(const void* data, size_t _size, vx::uint32 offset)
{
	VX_ASSERT(offset + _size <= size, "Trying to write out of bound data");
	GLCall(glNamedBufferSubData(iD, offset, _size, data));
}

void GraphicsBuffer::Orpan()
{
	glNamedBufferData(iD, size, nullptr, GL_DYNAMIC_DRAW);
}

void GraphicsBuffer::Destroy()
{
	GLCall(glDeleteBuffers(1, &iD)); iD = 0;
}

void GraphicsBuffer::AssignGPULabel(const char* name)
{
	if (iD)
	{
		vx::StackString<32> _name;
		_name << name;
		switch (type)
		{
		case BufferUsage::Vertex:
			_name << " VBO";
			//glObjectLabel(GL_BUFFER, iD, _name.Length(), _name.Data());
			//VX_INFO("Assigned GPU Label ", name, " VBO ", iD);
			break;
		case BufferUsage::Index:
			_name << " EBO";
			//glObjectLabel(GL_BUFFER, iD, (name.size() + 4), (name + " EBO").c_str());
			//VX_INFO("Assigned GPU Label ", name, " EBO ", iD);
			break;
		case /*Renderer::*/BufferUsage::Uniform:
			_name << " UBO";
			//glObjectLabel(GL_BUFFER, iD, (name.size() + 4), (name + " UBO").c_str());
			//VX_INFO("Assigned GPU Label ", name, " UBO ", iD);
			break;
		case BufferUsage::Storage:
			_name << " SSBO";
			//glObjectLabel(GL_BUFFER, iD, (name.size() + 5), (name + " SSBO").c_str());
			//VX_INFO("Assigned GPU Label ", name, " SSBO ", iD);
			break;
		}

		glObjectLabel(GL_BUFFER, iD, _name.Length(), _name.Data());
		VX_LOG_DEBUG("Assigned GPU Label ", _name.Data(), " with ID: ", iD);
	}
}
