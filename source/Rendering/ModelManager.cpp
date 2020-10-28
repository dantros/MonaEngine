#include "ModelManager.hpp"

#include <vector>
#include <cmath>
#include <glm/gtc/constants.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "../Core/Log.hpp"
#include <glad/glad.h>
namespace Mona {


	ModelManager::ModelHandle ModelManager::LoadModel(PrimitiveType type) const noexcept
	{
		switch (type)
		{
			case Mona::ModelManager::PrimitiveType::Plane:
			{
				return m_planeModel;
				break;
			}
			case Mona::ModelManager::PrimitiveType::Cube:
			{
				return m_cubeModel;
				break;
			}
			case Mona::ModelManager::PrimitiveType::Sphere:
			{
				return m_sphereModel;
				break;
			}
			default:
			{
				return m_sphereModel;
				break;
			}
		}
	}

	void ModelManager::StartUp() noexcept {
		Assimp::Importer importer;

		MONA_LOG_INFO("Testing Assimp : {0}", importer.IsExtensionSupported("OBJ")?"true" : "false");
		float vertices[] = {
	   -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f,
		1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f,
		1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f,
		1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f,
	   -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f,
	   -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f,

	   -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f,
		1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f,
		1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f,
		1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f,
	   -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f,
	   -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f,

	   -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f,
	   -1.0f,  1.0f, -1.0f, -1.0f,  0.0f,  0.0f,
	   -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f,
	   -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f,
	   -1.0f, -1.0f,  1.0f, -1.0f,  0.0f,  0.0f,
	   -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f,

		1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,
		1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f,
		1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,
		1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,
		1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,
		1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f,

	   -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f,
		1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f,
		1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f,
		1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f,
	   -1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f,
	   -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f,

	   -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f,
		1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f,
		1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f,
		1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f,
	   -1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f,
	   -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f
		};
		unsigned int indices[] = {
			0,1,2,3,4,5,
			6,7,8,9,10,11,
			12,13,14,15,16,17,
			18,19,20,21,22,23,
			24,25,26,27,28,29,
			30,31,32,33,34,35
		};
		unsigned int cubeVBO,cubeIBO, cubeVAO;
		glGenVertexArrays(1, &cubeVAO);
		glBindVertexArray(cubeVAO);
		
		glGenBuffers(1, &cubeVBO);
		glGenBuffers(1, &cubeIBO);
		glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeIBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		m_cubeModel = ModelHandle(cubeVAO, 36);
		
		float planeVertices[] = {
		-1.0f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
		1.0f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
		1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
		1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
		-1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
		-1.0f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
		};

		unsigned int planeIndices[] =
		{
			0,1,2,3,4,5
		};

		unsigned int planeVBO, planeIBO, planeVAO;
		glGenVertexArrays(1, &planeVAO);
		glBindVertexArray(planeVAO);
		glGenBuffers(1, &planeVBO);
		glGenBuffers(1, &planeIBO);

		glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, planeIBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(planeIndices), planeIndices, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		m_planeModel = ModelHandle(planeVAO, 6);


		LoadSphere();


	}

	void ModelManager::LoadSphere() noexcept {
		std::vector<float> vertices;
		std::vector<unsigned int> indices;
		unsigned int stackCount = 16;
		unsigned int sectorCount = 32;
		constexpr float PI = glm::pi<float>();
		float sectorStep = 2 * PI / sectorCount;
		float stackStep = PI / stackCount;
		float sectorAngle, stackAngle;
		float radius = 1.0f;

		float x, y, z, xy;
		for (unsigned int i = 0; i <= stackCount; i++)
		{
			stackAngle = PI / 2.0f - i * stackStep;
			xy = std::cos(stackAngle);             
			z = std::sin(stackAngle);
			for (unsigned int j = 0; j <= sectorCount; j++)
			{
				sectorAngle = sectorStep * j;
				x = xy * std::cos(sectorAngle);
				y = xy * std::sin(sectorAngle);
				vertices.push_back(radius * x);
				vertices.push_back(radius * y);
				vertices.push_back(radius * z);
				vertices.push_back(x);
				vertices.push_back(y);
				vertices.push_back(z);
			}

		}

		unsigned int k1, k2;
		for (unsigned int i = 0; i < stackCount; ++i)
		{
			k1 = i * (sectorCount + 1);     // beginning of current stack
			k2 = k1 + sectorCount + 1;      // beginning of next stack

			for (unsigned int j = 0; j < sectorCount; ++j, ++k1, ++k2)
			{
				if (i != 0)
				{
					indices.push_back(k1);
					indices.push_back(k2);
					indices.push_back(k1 + 1);
				}

				if (i != (stackCount - 1))
				{
					indices.push_back(k1 + 1);
					indices.push_back(k2);
					indices.push_back(k2 + 1);
				}
			}
		}
		unsigned int sphereVBO, sphereIBO, sphereVAO;
		glGenVertexArrays(1, &sphereVAO);
		glBindVertexArray(sphereVAO);

		glGenBuffers(1, &sphereVBO);
		glGenBuffers(1, &sphereIBO);
		glBindBuffer(GL_ARRAY_BUFFER, sphereVBO);
		glBufferData(GL_ARRAY_BUFFER, static_cast<unsigned int>(vertices.size())*sizeof(float), vertices.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sphereIBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<unsigned int>(indices.size()) * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
		m_sphereModel = ModelHandle(sphereVAO, indices.size());




	}

}