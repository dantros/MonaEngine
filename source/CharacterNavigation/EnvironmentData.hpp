#pragma once
#ifndef ENVIRONMENTDATA_HPP
#define ENVIRONMENTDATA_HPP

#include <vector>
#include <Eigen/Dense>

namespace Mona {
	typedef Eigen::Matrix<float, 1, 3> Vector3f;
	typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXf;
	typedef std::vector<unsigned int> Triangle;

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
			std::vector<unsigned int> m_orderedX;
			std::vector<unsigned int> m_orderedY;
			std::vector<Vector3f> m_vertices;
			std::vector<Triangle> m_triangles;
			// Asociamos cada indice a los triangulos en los que esta incluido
			std::unordered_map<unsigned int, std::vector<Triangle*>> m_triangleMap;

		public:
			MeshData() = default;
			int getID() { return m_id; }
			bool withinBoundaries(float x, float y);
			int orientationTest(Vector3f v1, Vector3f v2, Vector3f testV); // se ignora coordenada z
			bool triangleContainsPoint(Triangle t, Vector3f p); // se ignora coordenada z
			void init(std::vector<Vector3f>& vertices, std::vector<Triangle>& faces);
			bool isValid() { return m_isValid; }
	};
	int MeshData::lastId = 0;

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