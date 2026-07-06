#pragma once

#include "SampleFramework/Renderer/GPUVertexData.h"

#include "Vortrix/Maths/VortrixMaths.h"
#include <vector>

namespace Util
{
	constexpr double PI = 3.141592653589793238462643;

	static inline RenderableMesh CreateSphere(unsigned int sector_count = 36, unsigned int span_count = 18)
	{
		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;

		//vextex position
		float x, y, z, w;
		//vectex texture coord
		float u, v;

		float sector_angle;
		float span_angle;

		//TO-DO: for now use float, double will be too redundant to use 
		float sector_step = (float)(2 * PI / sector_count);  // 0 - 360(2pi)/count =>  angle btw each steps
		float span_step = (float)(PI / span_count);		  // 0 - 180(pi)/count => angle btw each step

		//float radius = 1.0f;
		float radius = 0.5f;

		//compute & store vertices
		for (unsigned int i = 0; i <= span_count; i++)
		{
			// 180 degree(pi) to 0 degree //0 degree to 180 degree(pi)
			span_angle = (float)PI - i * span_step;

			//parametric equation for sphere
			// x = center_x + r * sin(theta) * cos(phi)    
			// y = center_y + r * sin(thata) * sin(phi)
			// z = center_z + r * cos(theta)
			// where 
			//		theta = span_angle range 0 to pi(180 degrees)
			//		phi = sector_angle range 0 to 2pi(360 degrees)
			//RETERIVED: https://en.m.wikipedia.org/wiki/Sphere
			//			In their case z is up axis


			w = radius * vx::VxSin(span_angle);
			y = radius * vx::VxCos(span_angle);

			//add (sector_count + 1) vertices per stack
			//first and last vertices have same position, but different tex coords 
			for (unsigned int j = 0; j <= sector_count; ++j)
			{
				//from from 0 >> current step * step angle >> 360 
				sector_angle = j * sector_step;

				//vectex position (x, y, z)
				x = w * vx::VxCos(sector_angle);			//r * cos(u) * cos(v)
				z = w * vx::VxSin(sector_angle);			//r * cos(u) * sin(v)

				//vertex texture coord range between [0, 1]
				u = (float)j / sector_count;
				v = (float)i / span_count;

				Vertex vertex
				{
					{x,y, z},
					{x,y, z},
					{u,v},
					{x,y, z}
				};
				vertices.push_back(vertex);
			}
		}


		//compute & store indices
		unsigned int k1, k2;
		for (unsigned int i = 0; i < span_count; ++i)
		{
			k1 = i * (sector_count + 1);		//beginning of current stack
			k2 = k1 + sector_count + 1;			//beginning of next stack

			for (unsigned int j = 0; j < sector_count; ++j, ++k1, ++k2)
			{
				//2 triangles per sector excluding first and last stacks
				//k1 => k2 => k1+1
				if (i != 0)
				{
					indices.push_back(k1);
					indices.push_back(k2);
					indices.push_back(k1 + 1);
				}

				//k1+1 => k2 => k2+ 1
				if (i != (span_count - 1))
				{
					indices.push_back(k1 + 1);
					indices.push_back(k2);
					indices.push_back(k2 + 1);
				}

			}
		}

		return RenderableMesh(vertices, indices);
	}



	//static inline void RecursiveSubdivide(vx::Vec3 v0, vx::Vec3 v1, vx::Vec3 v2, int depth, vx::Vec3 offset, const void* vertices_data, int& vertex_count)
	static inline void RecursiveSubdivide(vx::Vec3 v0, vx::Vec3 v1, vx::Vec3 v2, int depth, 
		vx::Vec3 offset, std::vector<Vertex>& vertices_data, int& triangle_count)
	{
		struct _Vertex
		{
			vx::Vec3 position;
			vx::Vec3 normal;
			vx::Vec2 uv;
			vx::Vec3 colour;   //<-----for debug/visuals
		};

		//helper
		auto ToVertex = [&](const _Vertex& v)
			{
				Vertex vertex
				{
					{v.position[0], v.position[1], v.position[2]},
					{v.normal[0], v.normal[1], v.normal[2]},
					{v.uv[0], v.uv[1]},
					{v.colour[0], v.colour[1], v.colour[2]}
				};
				return vertex;
			};

		if (depth == 0)
		{
			vx::Vec3 pos[] = { v0, v1, v2 };

			float h_half = vx::VxAbs(offset.Y());

			_Vertex v[3];
			for (int i = 0; i < 3; i++)
			{
				v[i].position = pos[i] + offset;
				vx::Vec3 n = pos[i].Normalised();
				v[i].normal = n;

				//float radius = glm::length(pos[i]);
				//float span_half = h_half + radius;

				//longtitude atan2 range[-PI, PI] map [0, 1]
				float u = 0.5f + (vx::VxAtan2(n.Z(), n.X()) / (2.0f * vx::kVxPi));
				//float u = 0.5f + (atan2(n.z, n.x) / (2.0f * (float)M_PI));
				//latitude asin range [-PI/2, PI/2] map [0, 1]
				float _v = 0.5f + (vx::VxAsin(n.Y()) / vx::kVxPi);
				//float _v = (v[i].position.y + span_half) / (span_half * 2.0f);

				v[i].uv = vx::Vec2(u, _v);

			}


			//fix
			if (vx::VxAbs(v[0].uv.X() - v[1].uv.X()) > 0.5f ||
				vx::VxAbs(v[1].uv.X() - v[2].uv.X()) > 0.5f ||
				vx::VxAbs(v[1].uv.X() - v[0].uv.X()) > 0.5f)
			{
				for (int i = 0; i < 3; i++)
					if (v[i].uv.X() < 0.5f)
						v[i].uv[0] += 1.0f;
			}

			vertices_data.push_back(ToVertex(v[0]));
			vertices_data.push_back(ToVertex(v[1]));
			vertices_data.push_back(ToVertex(v[2]));
			triangle_count++;
			return;

			//mVBO.AddVertexData(&v, sizeof(Vertex)*3);
			//mVBO.AddVertexData(&v, sizeof(v));
			//mIndicesCount++;
			//return;
		}


		//target radius 
		float radius = v0.Length();
		//mid points
		vx::Vec3 v01 = (v0 + v1).Normalised() * radius;
		vx::Vec3 v12 = (v1 + v2).Normalised() * radius;
		vx::Vec3 v20 = (v2 + v0).Normalised() * radius;

		RecursiveSubdivide(v0, v01, v20, depth - 1, offset, vertices_data, triangle_count);
		//RecursiveSubdivide(v1, v12, v01, depth - 1, offset);
		RecursiveSubdivide(v01, v1, v12, depth - 1, offset, vertices_data, triangle_count);
		//RecursiveSubdivide(v2, v20, v12, depth - 1, offset);
		RecursiveSubdivide(v20, v12, v2, depth - 1, offset, vertices_data, triangle_count);
		RecursiveSubdivide(v01, v12, v20, depth - 1, offset, vertices_data, triangle_count);
	}


	static inline RenderableMesh CreateOctaSphere(int subdivisions = 4)
	{
		vx::Vec3 vertex[6] =
		{
			{0.0f, 0.5f, 0.0f},//Top
			{0.0f, -0.5f, 0.0f},//Botttom
			{0.5f, 0.0f, 0.0f},//Right
			{-0.5f, 0.0f, 0.0f},//Left
			{0.0f, 0.0f, 0.5f},//Front
			{0.0f, 0.0f, -0.5f},//Back
		};

		//glm::vec3 offset = glm::vec3(0.0f, mCylinderHeight * 0.5f, 0.0f);
		vx::Vec3 offset = vx::Vec3::Zero();


		std::vector<Vertex> vertices_data;
		int triangle_count = 0;

		RecursiveSubdivide(vertex[0], vertex[4], vertex[2], subdivisions, offset, vertices_data, triangle_count);
		RecursiveSubdivide(vertex[0], vertex[3], vertex[4], subdivisions, offset, vertices_data, triangle_count);
		RecursiveSubdivide(vertex[0], vertex[5], vertex[3], subdivisions, offset, vertices_data, triangle_count);
		RecursiveSubdivide(vertex[0], vertex[2], vertex[5], subdivisions, offset, vertices_data, triangle_count);

		RecursiveSubdivide(vertex[1], vertex[2], vertex[4], subdivisions, -offset, vertices_data, triangle_count);
		RecursiveSubdivide(vertex[1], vertex[4], vertex[3], subdivisions, -offset, vertices_data, triangle_count);
		RecursiveSubdivide(vertex[1], vertex[3], vertex[5], subdivisions, -offset, vertices_data, triangle_count);
		RecursiveSubdivide(vertex[1], vertex[5], vertex[2], subdivisions, -offset, vertices_data, triangle_count);




		//RecursiveSubdivide(vertex[0], vertex[2], vertex[4], subdivisions, offset, vertices_data, triangle_count);
		//RecursiveSubdivide(vertex[0], vertex[4], vertex[3], subdivisions, offset, vertices_data, triangle_count);
		//RecursiveSubdivide(vertex[0], vertex[3], vertex[5], subdivisions, offset, vertices_data, triangle_count);
		//RecursiveSubdivide(vertex[0], vertex[5], vertex[2], subdivisions, offset, vertices_data, triangle_count);

		//RecursiveSubdivide(vertex[1], vertex[4], vertex[3], subdivisions, -offset, vertices_data, triangle_count);
		//RecursiveSubdivide(vertex[1], vertex[3], vertex[4], subdivisions, -offset, vertices_data, triangle_count);
		//RecursiveSubdivide(vertex[1], vertex[5], vertex[3], subdivisions, -offset, vertices_data, triangle_count);
		//RecursiveSubdivide(vertex[1], vertex[2], vertex[5], subdivisions, -offset, vertices_data, triangle_count);

		return RenderableMesh(vertices_data, triangle_count);
	}



	static inline RenderableMesh CreateCapsule(float height, float radius, int sphere_depth)
	{
		int cylinder_segments = 4 * vx::VxPow(2, sphere_depth);
		std::vector<Vertex> vertices_data;
		int triangle_count = 0;


		auto AddHemisphere = [&](float y_offset, bool top)
			{
				vx::Vec3 v[6] = {
					vx::Vec3(1, 0, 0),
					vx::Vec3(-1, 0, 0),
					vx::Vec3(0, 1, 0),
					vx::Vec3(0, -1, 0),
					vx::Vec3(0, 0, 1),
					vx::Vec3(0, 0, -1)
				};

				int f[8][3] = {
					{0, 2, 4}, {4, 2, 1},
					{1, 2, 5}, {5, 2, 0},

					{0, 4, 3}, {4, 1, 3},
					{1, 5, 3}, {5, 0, 3}
				};

				vx::Vec3 offset = vx::Vec3(0.0f, y_offset, 0.0f);
				for (int i = 0; i < 8; i++)
				{
					vx::Vec3 a = v[f[i][0]].Normalise() * radius;
					vx::Vec3 b = v[f[i][1]].Normalise() * radius;
					vx::Vec3 c = v[f[i][2]].Normalise() * radius;

					///cull bottom hemisphere for top, top for bottom
					if (top && (a.Y() < 0.0f || b.Y() < 0.0f || c.Y() < 0.0f))continue;
					if (!top && (a.Y() > 0.0f || b.Y() > 0.0f || c.Y() > 0.0f))continue;

					RecursiveSubdivide(a, b, c, sphere_depth, offset, vertices_data, triangle_count);
				}
			};

		AddHemisphere(height * 0.5f, true);
		AddHemisphere(-height * 0.5f, false);

		struct _Vertex
		{
			vx::Vec3 position;
			vx::Vec3 normal;
			vx::Vec2 uv;
			vx::Vec3 colour;   //<-----for debug/visuals
		};

		//helper
		auto ToVertex = [&](const _Vertex& v)
			{
				Vertex vertex
				{
					{v.position[0], v.position[1], v.position[2]},
					{v.normal[0], v.normal[1], v.normal[2]},
					{v.uv[0], v.uv[1]},
					{v.colour[0], v.colour[1], v.colour[2]}
				};
				return vertex;
			};

		for (int i = 0; i < cylinder_segments; i++)
		{
			float theta_0 = (2.0f * vx::kVxPi * i / cylinder_segments);
			float theta_1 = (2.0f * vx::kVxPi * (i + 1) / cylinder_segments);


			vx::Vec3 p0 = vx::Vec3(radius * vx::VxCos(theta_0), -height / 2, radius * vx::VxSin(theta_0));
			vx::Vec3 p1 = vx::Vec3(radius * vx::VxCos(theta_1), -height / 2, radius * vx::VxSin(theta_1));
			vx::Vec3 p2 = vx::Vec3(radius * vx::VxCos(theta_1), height / 2, radius * vx::VxSin(theta_1));
			vx::Vec3 p3 = vx::Vec3(radius * vx::VxCos(theta_0), height / 2, radius * vx::VxSin(theta_0));

			//nor
			vx::Vec3 n0 = vx::Vec3(p0.X(), 0.0f, p0.Z()).Normalise();
			vx::Vec3 n1 = vx::Vec3(p1.X(), 0.0f, p1.Z()).Normalise();
			vx::Vec3 n2 = vx::Vec3(p2.X(), 0.0f, p2.Z()).Normalise();
			vx::Vec3 n3 = vx::Vec3(p3.X(), 0.0f, p3.Z()).Normalise();


			float total_height = height + (2.0f * radius);
			float v_bottom = radius / total_height;
			float v_top = (radius + height) / total_height;
			//uv
			vx::Vec2 uv0(i / (float)cylinder_segments, v_bottom);
			vx::Vec2 uv1((i + 1) / (float)cylinder_segments, v_bottom);
			vx::Vec2 uv2((i + 1) / (float)cylinder_segments, v_top);
			vx::Vec2 uv3(i / (float)cylinder_segments, v_top);

			//Vertex quad[6] = {
			//	ToVertex({p0, n0, uv0}),
			//	ToVertex({p2, n2, uv2}),
			//	ToVertex({p1, n1, uv1}),

			//	ToVertex({p0, n0, uv0}),
			//	ToVertex({p3, n3, uv3}),
			//	ToVertex({p2, n2, uv2}),
			//};


			vertices_data.push_back(ToVertex({p0, n0, uv0})); 
			vertices_data.push_back(ToVertex({p2, n2, uv2}));
			vertices_data.push_back(ToVertex({p1, n1, uv1}));

			vertices_data.push_back(ToVertex({p0, n0, uv0}));
			vertices_data.push_back(ToVertex({p3, n3, uv3}));
			vertices_data.push_back(ToVertex({p2, n2, uv2}));

			//mVBO.AddVertexData(quad, sizeof(quad));
			triangle_count += 2;
		}
		return RenderableMesh(vertices_data, triangle_count);
	}


	namespace Random
	{
		//quick random value between two value, min & max
		static inline float Float(float min, float max)
		{
			return min + static_cast<float>(rand()) / (static_cast<float>(RAND_MAX / (max - min)));
		}


		//quick random value between two value, min & max
		static inline int Int(int min, int max)
		{
			return min + rand() / (RAND_MAX / (max - min));
		}


		//quick random point but not fully uniform 
		static inline vx::Vec3 PointInSphere(float radius)
		{
			if (radius <= 0.0f)
				return vx::Vec3(0.0f);

			while (true)
			{
				float x = Float(-radius, radius);
				float y = Float(-radius, radius);
				float z = Float(-radius, radius);

				//printf("Random point in sphere x: %f, y: %f, z: %f\n", x, y, z);

				if (x * x + y * y + z * z <= radius * radius)
					return vx::Vec3(x, y, z);
			}
		}
	}//Random namespace


} //Util namespace




