#pragma once
#include <vector>

struct Vertex
{
	float position[3];
	float normal[3];   
	float uv[2];
	float colour[3];   //<-----for debug/visuals
};

class RenderableMesh
{
private:
	unsigned int mVAO = 0;
	unsigned int mVBO = 0;
	unsigned int mIBO = 0;
public:
	RenderableMesh() = default;

	RenderableMesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices);
	RenderableMesh(const std::vector<Vertex>& vertices, unsigned int triangle_count);
	void Create(const std::vector<Vertex>& vertices, std::vector<unsigned int> indices);
	void Create(const std::vector<Vertex>& vertices, unsigned int triangle_count);

	void Bind();
	void UnBind();

	void Draw();
	void DrawOutline();

	void Clear();
	~RenderableMesh() = default;

private:
	unsigned int m_IndiceCount = 0;
	unsigned int mTriangleCount = 0;
};

#include <Vortrix/Core/Colours.h>

struct Light
{
	vx::Colour colour = vx::Colour(vx::Vec3(1.0f), 1.0f);
	float intensity = 0.7f;
};

struct DirectionalLight : public Light
{
	vx::Vec3 direction = vx::Vec3(0.0f, -1.0f, 0.0f);
};


class ShadowMap
{
public:
	ShadowMap() = default;
	ShadowMap(unsigned int in_size);
	void Generate();
	void Write();
	virtual void Read(unsigned int slot = 0);
	inline unsigned int GetColourAttachment() { return mColourAttachment; }

	void Destroy();
private:
	unsigned int mSize = 4096;
	unsigned int mID = 0;
	unsigned int mColourAttachment = 0;
};

#include <functional>
#include "Vortrix/Geometry/AABB.h"
struct ShadowData
{
	vx::Vec3 origin = vx::Vec3::Zero(); //centered at origin, unless modified
	float distance = 1.0f;
	float zNearOffset = 0.1f;

	float split_depth = 0.17f;
	vx::AABB orthographicBounds = vx::AABB(0.5f);

	float instantousFar = 100.0f;

	vx::Mat44 ComputeLightView(const vx::Vec3& light_dir, vx::Vec3 world_up = vx::Vec3::Up())
	{
		return ShadowData::ComputeLightView(GetOrigin(origin), light_dir, distance, world_up);
	}

	vx::Mat44 ComputeProjection(const vx::Mat44& light_view_mat);

	vx::Mat44 ComputeProjectionViewMat(const vx::Vec3& light_dir, vx::Vec3 world_up = vx::Vec3::Up())
	{
		const vx::Mat44& view_mat = ComputeLightView(light_dir, world_up);
		return ComputeProjection(view_mat).Multiply(view_mat);
	}


	static vx::Mat44 ComputeLightView(const vx::Vec3& _origin,
		const vx::Vec3& light_dir, float dist, vx::Vec3 world_up = vx::Vec3::Up());

	vx::Vec3 GetOrigin(const vx::Vec3& fallback)
	{
		return originQueryCB ? originQueryCB(this) : /*origin*/fallback;
	}
	bool HasOriginQuery() const { return originQueryCB.operator bool(); }

	using OriginQueryCallback = std::function<vx::Vec3(ShadowData* shadow_data)>;
	void SetOriginQuery(const OriginQueryCallback& func) { originQueryCB = func; }

	using OrthoQueryCallback = std::function<void(const vx::Mat44& light_mat, ShadowData* shadow_data)>;
	void SetOrthoQuery(const OrthoQueryCallback& func) { orthoQueryCB = func; }
private:
	OriginQueryCallback originQueryCB;
	OrthoQueryCallback orthoQueryCB;
};

