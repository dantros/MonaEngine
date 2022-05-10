#pragma once
#ifndef ENVIRONMENTDATA_HPP
#define ENVIRONMENTDATA_HPP

#include <vector>
#include <Eigen/Dense>

namespace Mona {
	typedef Eigen::Matrix<float, 1, 3> Vector3f;
	typedef Eigen::Matrix<float, 1, 3> Vector2f;
	typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXf;

	struct Triangle {
		std::vector<Vertex> vertices;
		std::vector<Triangle*> neighbors;
	};

	struct Vertex {
		float x;
		float y;
		float z;
	};

	class MeshData{
		friend class EnvironmentData;
		private:
			static int lastId;
			int m_id;
			bool m_isValid = false;
			float m_minX;
			float m_minY;
			float m_maxX;
			float m_maxY;
			std::vector<Triangle> m_triangles;

		public:
			MeshData() = default;
			int getID() { return m_id; }
			bool withinBoundaries(float x, float y);
			int orientationTest(Vertex v1, Vertex v2, Vertex testV); // se ignora coordenada z
			bool triangleContainsPoint(Triangle t, Vertex p); // se ignora coordenada z
			void orderVerticesCCW(std::vector<Vertex>* vertices);
			void orderTriangle(Triangle* t);
			void init(std::vector<Vector3f>& vertices, std::vector<std::vector<unsigned int>>& faces);
			bool isValid() { return m_isValid; }
	};

	class EnvironmentData {
		private:
			std::vector<MeshData> m_terrains;
			std::vector<MeshData> m_obstacles;
			float _getTerrainHeight(float x, float y, MeshData terrain);
		public:
			float getTerrainHeight(float x, float y);
			void addTerrain(MeshData& terrain);
			int removeTerrain(MeshData& terrain);


	};

}

#endif