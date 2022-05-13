#include "CharacterNavigation/HeightMap.hpp"
#include <glm/glm.hpp>
#include <numbers>
#include <iostream>

namespace Mona {
	void generateTerrain(const glm::vec2& bottomLeft, const glm::vec2& topRight, int numInnerVerticesWidth, int numInnerVerticesHeight,
		float (*heightFunc)(float, float), HeightMap* heightMap=nullptr)
	{
		// v = {pos_x, pos_y, pos_z, normal_x, normal_y, normal_z, color_x, color_y, color_z,}
		std::vector<float> vertices;
		std::vector<unsigned int> faces;
		size_t numVertices = 0;
		size_t numFaces = 0;

		float stepX = (topRight[0] - bottomLeft[0]) / (numInnerVerticesWidth + 1);
		float stepY = (topRight[1] - bottomLeft[1]) / (numInnerVerticesHeight + 1);
		for (int i = 0; i < numInnerVerticesWidth + 2; i++) {
			float x = bottomLeft[0] + stepX * i;
			for (int j = 0; j < numInnerVerticesHeight + 2; j++) {
				float y = bottomLeft[1] + stepY * j;
				float z = heightFunc(x, y);
				numVertices += 1;
				vertices.insert(vertices.end(), { x, y, z });
			}
		}

		// indexing func
		auto index = [&](int i, int j)
		{
			return i * (numInnerVerticesHeight + 2) + j;
		};

		// We generate quads for each cell connecting 4 neighbor vertices
		for (int i = 0; i < numInnerVerticesWidth + 1; i++) {
			for (int j = 0; j < numInnerVerticesHeight + 1; j++) {
				// Getting indices for all vertices in this quad
				unsigned int isw = index(i, j);
				unsigned int ise = index(i + 1, j);
				unsigned int ine = index(i + 1, j + 1);
				unsigned int inw = index(i, j + 1);

				numFaces += 2;
				faces.insert(faces.end(), { isw, ise, ine, ine, inw, isw }); // falta rellenar las normales
			}
		}


		if (heightMap != nullptr) {
			std::vector<Vector3f> vertexPositions;
			std::vector<Vector3ui> groupedFaces;
			vertexPositions.reserve(numVertices);
			groupedFaces.reserve(numFaces);
			for (int i = 0; i < numInnerVerticesWidth + 2; i++) {
				for (int j = 0; j < numInnerVerticesHeight + 2; j++) {
					Vector3f v(vertices[index(i, j)], vertices[index(i, j + 1)], vertices[index(i, j + 2)]);
					vertexPositions.push_back(v);
				}
			}
			for (int i = 0; i < faces.size(); i += 3) {
				Vector3ui f = { faces[i], faces[i + 1], faces[i + 2] };
				groupedFaces.push_back(f);
			}
			heightMap->init(vertexPositions, groupedFaces);


		}

	}

}

float gaussian(float x, float y, float s, float sigma, glm::vec2 mu) {
	return (s / (sigma * std::sqrt(2 * std::numbers::pi))) * std::exp((-1 / (2 * std::pow(sigma, 2))) * (std::pow((x - mu[0]), 2) + std::pow((y - mu[1]), 2)));
}

int main()
{

	auto heightFun = [](float x, float y) {
		return (gaussian(x, y, 30, 5, { -10, 0 }) + gaussian(x, y, 50, 3, { 10, 0 }));
	};
	Mona::HeightMap hm;
	Mona::generateTerrain({ 0,0 }, { 10,10 }, 10, 20, heightFun, &hm);

	std::cout << "altura: " << hm.getHeight(5, 5);
	
	return 0;

}
