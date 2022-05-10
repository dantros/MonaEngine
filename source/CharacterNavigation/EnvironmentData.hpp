#pragma once
#ifndef ENVIRONMENTDATA_HPP
#define ENVIRONMENTDATA_HPP

#include <vector>
#include <Eigen/Dense>

namespace Mona {
	typedef Eigen::Matrix<float, 1, 3> Vector3f;
	typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXf;

	struct IndexedVertex {
		int index;
		Vector3f vertex;
		static bool compareX(IndexedVertex& v1, IndexedVertex& v2) { return v1.vertex[0] < v2.vertex[0]; }
		static bool compareY(IndexedVertex& v1, IndexedVertex& v2) { return v1.vertex[1] < v2.vertex[1]; }
	};

	class VertexData{
		friend class EnvironmentData;
		private:
			static int lastId;
			int m_id;
			float m_minX;
			float m_minY;
			float m_maxX;
			float m_maxY;
			std::vector<int> m_orderedX;
			std::vector<int> m_orderedY;
			std::vector<Vector3f> m_vertexArray;

		public:
			VertexData() = delete;
			VertexData(std::vector<Vector3f> vArray);
			int getID() { return m_id; }
	};
	int VertexData::lastId = 0;

	class EnvironmentData {
		private:
			std::vector<VertexData> m_terrains;
			std::vector<VertexData> m_obstacles;
			float getTerrainHeight(float x, float y, VertexData terrain);
		public:
			void addTerrain(VertexData& terrain);
			int removeTerrain(VertexData& terrain);


	};

}

#endif