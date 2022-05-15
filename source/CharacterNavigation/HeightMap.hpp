#pragma once
#ifndef HEIGHTMAP_HPP
#define HEIGHTMAP_HPP

#include <vector>
#include <Eigen/Dense>



namespace Mona {
    
	typedef Eigen::Matrix<float, 1, 3> Vector3f;
	typedef Eigen::Matrix<float, 1, 2> Vector2f;
	typedef Eigen::Matrix<unsigned int, 1, 3> Vector3ui;
	typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXf;
	typedef Eigen::Matrix<float, 1, 3> Vertex;
	typedef unsigned int vIndex;

	struct Triangle {
		std::vector<vIndex> vertices;
		std::vector<Triangle*> neighbors = { nullptr, nullptr, nullptr };
		friend bool operator!= (const Triangle& t1, const Triangle& t2);
		friend bool operator== (const Triangle& t1, const Triangle& t2);
	};

	class HeightMap{
		friend class EnvironmentData;
		private:
			static unsigned int lastId;
			unsigned int m_id;
			bool m_isValid = false;
			float m_minX;
			float m_minY;
			float m_maxX;
			float m_maxY;
			float m_epsilon = 0.000001;
			std::vector<vIndex> m_orderedX;
			std::vector<vIndex> m_orderedY;
			std::vector<Triangle> m_triangles;
			std::vector<Vertex> m_vertices;
			std::unordered_map<vIndex, std::vector<Triangle*>> m_triangleMap;
			float (*m_heightFunc)(float, float) = nullptr;
			int orientationTest(const Vertex& v1, const Vertex& v2, const Vertex& testV); // se ignora coordenada z
			bool triangleContainsPoint(Triangle* t, const Vertex& p); // se ignora coordenada z
			int sharedVertices(Triangle* t1, Triangle* t2);
			bool orderVerticesCCW(std::vector<vIndex>* vertices);
			bool orderTriangle(Triangle* t);
			bool goesThroughTriangle(vIndex start, Vertex end, Triangle* triangle);
			Triangle* nextTriangle(Vertex start, Vertex end, Triangle* triangle);
			float getInterpolatedHeight(Triangle* t, float x, float y);
			vIndex findCloseVertex(float x, float y);

		public:
			HeightMap() = default;
			unsigned int getID() { return m_id; }
			bool withinBoundaries(float x, float y);
			void init(const std::vector<Vector3f>& vertices, const std::vector<Vector3ui>& faces);
			void init(const Vector2f& bottomLeft, const Vector2f& topRight, float (*heightFunc)(float, float));
			bool isValid() { return m_isValid; }
			float getHeight(float x, float y);
	};

}


#endif