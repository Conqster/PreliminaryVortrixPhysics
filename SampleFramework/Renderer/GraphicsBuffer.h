#pragma once


#include "ErrorAssertion.h"

#include "Vortrix/Core/NonCopyable.h"
#include "Vortrix/Core/Core.h"

struct VertexAttribute
{
	enum class Type : vx::uint8 { Float, UInt8, Int32, UInt32 };

	vx::uint32 index = 0;
	vx::uint32 size = 0;
	Type type = Type::Float;
	bool normalised = false;
	uint32_t offset = 0;
};

enum class BufferUsage : vx::uint16// < --becomes usage
{
	Vertex,
	Index,
	Uniform,
	Storage
};

enum class BufferMemory : vx::uint8
{
	DeviceLocal, //most efficient, only visible to GPU
	HostVisible,  //CPU to GPU. for freq uploads (instancing)
	HostCached,		//GPU to CPU, for reading result back 
	HostPersistent,			//Zero copy, map once keep pointer
};

enum class BufferFrequency : vx::uint8
{
	Static, //Once at startup (scenario geometry)
	Dynamic, //Once per frame (camera / unifoirm) 
	Stream  //multiple times per frame (substepping/physics gizmos)
};


/// Vertex -> static/baked geometries -> DeviceLocal + Static (GL static draw) 
/// Index -> static/baked geometries -> DeviceLocal + Static (GL static draw) 
/// Uniform -> freq update mid data, most case CPU_GPU + dynamic/stream 
/// Storage -> compute/physics/instancing -> DeviceLocal/CPU_GPU + stream
static constexpr GLenum InferGLUsage(BufferMemory memory, BufferFrequency freq)
{
	switch (memory)
	{
	case BufferMemory::DeviceLocal: return GL_STATIC_DRAW;

	case BufferMemory::HostVisible:
		return (freq == BufferFrequency::Stream) ? GL_STREAM_DRAW : GL_DYNAMIC_DRAW;

	case BufferMemory::HostCached:
		return (freq == BufferFrequency::Stream) ? GL_STREAM_READ : GL_DYNAMIC_READ;

	case BufferMemory::HostPersistent:
		return GL_DYNAMIC_READ;
	}
	return GL_DYNAMIC_READ;
}




struct GraphicsBuffer : vx::NonCopyable
{
	vx::uint32 iD = 0;
	BufferUsage type = BufferUsage::Vertex;
	size_t count = 0;
	size_t size = 0;

	bool IsValid() const { return (iD != 0); }

	static void CreateBuffers(uint32_t* ids, int count)
	{
		glCreateBuffers(count, ids);
	}

	void Generate(BufferUsage _type, size_t _size, const void* data = nullptr,
		BufferMemory mem = BufferMemory::HostVisible, BufferFrequency usage_freq = BufferFrequency::Dynamic);

	/// data stride, should not be confused with vertex stride 
	/// vertex stride is important when it comes to BindingVertex layout
	void Generate(BufferUsage _type, GLsizei data_stride, uint32_t _count, const void* data,
		BufferMemory mem = BufferMemory::HostVisible, BufferFrequency usage_freq = BufferFrequency::Dynamic);


	void BindVertexLayout(const VertexAttribute* attrib, int count, const uint32_t VAO, uint32_t stride);

	void Bind(uint32_t slot_point = 0) const;

	//static void UnBind() {glBindBuffer}

	//binding specifi ranges [must 256-bytes aligned offset]
	void SegmentBind(size_t size, vx::uint32 slot_point = 0, vx::uint32 offset = 0) const;

	void ValidateSize(size_t bytes_needed, const void* data = nullptr);

	

	void ValidateSize(size_t bytes_needed, const void* data,
		BufferMemory mem = BufferMemory::HostVisible, BufferFrequency usage_freq = BufferFrequency::Dynamic);

	void Upload(const void* data, vx::uint32 offset = 0);

	void SegmentUpload(const void* data, size_t _size, vx::uint32 offset = 0);

	void Orpan(); /*Reset()*/


	void Destroy();

	size_t AlignOffset(size_t offset) const
	{

		static GLint ubo_alignment = 0;
		glGetIntegerv(GL_UNIFORM_BUFFER_OFFSET_ALIGNMENT, &ubo_alignment);
		static GLint ssbo_alignment = 0;
		glGetIntegerv(GL_SHADER_STORAGE_BUFFER_OFFSET_ALIGNMENT, &ssbo_alignment);

		size_t alignment = 16;
		if (type == BufferUsage::Uniform)
			alignment = ubo_alignment;
		else if (type == BufferUsage::Storage)
			alignment = ssbo_alignment;

		//round up bitwise math
		return (offset + alignment - 1) & ~(alignment - 1);
	}


	//void AssignGPULabel(const std::string& name)
	void AssignGPULabel(const char* name);
};




struct RingBuffer : public GraphicsBuffer
{
	size_t currentOffset = 0;


	/// PushData
	/// takes the passing data along with bytes needed to accompaned the data
	/// 
	/// and returns the offset where the data sits in bytes.
	size_t PushData(const void* data, size_t bytes_needed)
	{
		currentOffset = AlignOffset(currentOffset); //current start to align with cache line

		if (currentOffset + bytes_needed > size)
		{
			Orpan();
			currentOffset = 0;
		}

		SegmentUpload(data, bytes_needed, uint32_t(currentOffset));

		/// this to be able to be used with VBO etxc
		/// to use for first draw vertex
		uint32_t start_offset = uint32_t(currentOffset);
		currentOffset += bytes_needed;

		return start_offset;
	}
};