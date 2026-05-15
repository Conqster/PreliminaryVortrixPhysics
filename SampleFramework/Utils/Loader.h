#pragma once

#include "Renderer/GPUVertexData.h"

#include <string>
#include <fstream>
#include <sstream>

#define DEBUG_TO_CONSOLE 0


#if DEBUG_TO_CONSOLE
	#include <iostream>
	#define CONSOLE_LOG(x) std::cout << x << std::endl
#else
	#define CONSOLE_LOG(x)
#endif // DEBUG_TO_CONSOLE



namespace Loader
{

	inline RenderableMesh LoadMesh(const std::string& path)
	{
		std::ifstream file_stream(path, std::ios::in);
		if (!file_stream.is_open())
		{
			CONSOLE_LOG("[Loader Mesh]: Failed to open file: " << path.c_str());
			return {};
		}

		std::vector<Vertex> vertices;
		std::vector<unsigned int> indices;
		int has_colour = 0;
		std::string line;
		while (std::getline(file_stream, line))
		{
			std::istringstream iss(line);
			std::string prefix;
			iss >> prefix;
			CONSOLE_LOG(prefix.c_str());
			if (prefix == "vcount")
			{
				int c;
				iss >> c;
				vertices.reserve(c);
				CONSOLE_LOG("vertex count: " << c);
			}
			else if (prefix == "hascolour")
			{
				iss >> has_colour;
				CONSOLE_LOG("vertex has colour: " << ((has_colour)? "true":"false"));
			}
			else if (prefix == "icount")
			{
				int c;
				iss >> c;
				indices.reserve(c);
				CONSOLE_LOG("indices count: " << c);
			}
			else if (prefix == "v")
			{
				Vertex v;
				iss >> v.position[0] >> v.position[1] >> v.position[2] >> 
					   v.normal[0] >> v.normal[1] >> v.normal[2] >> 
					   v.uv[0] >> v.uv[1];
				if(has_colour)
					iss >> v.colour[0] >> v.colour[1] >> v.colour[2];
				else
				{
					v.colour[0] = v.position[0];
					v.colour[1] = v.position[1];
					v.colour[2] = v.position[2];
				}
					
				vertices.push_back(v);
				CONSOLE_LOG("vertex x: " << v.position[0] << "f, y: " << v.position[1] << "f, z : " << v.colour[0] << "f, r : " << v.colour[1] << "f, g : " << v.colour[1] << "f, b : " << v.colour[2] << "f");
			}
			else if (prefix == "i")
			{
				unsigned int i[3];
				iss >> i[0] >> i[1] >> i[2];
				indices.push_back(i[0]);
				indices.push_back(i[1]);
				indices.push_back(i[2]);
				CONSOLE_LOG("triange indices: " << i[0] << ", " << i[1] << ", " << i[2]);
			}
		}

		RenderableMesh renderable_mesh(vertices, indices);
		return renderable_mesh;
	}








}//Loader namespace