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
	typedef unsigned int vIndex;
	typedef int vNum;

	struct Triangle {
		std::vector<vIndex> vertices;
		std::vector<Triangle*> neighbors;
		friend bool operator!= (const Triangle& t1, const Triangle& t2);
		friend bool operator== (const Triangle& t1, const Triangle& t2);
	};

	struct Vertex {
		float x;
		float y;
		float z;
		friend bool operator!= (const Vertex& v1, const Vertex& v2);
		friend bool operator== (const Vertex& v1, const Vertex& v2);
	};

	class HeightMap{
		friend class EnvironmentData;
		private:
			static int lastId;
			int m_id;
			bool m_isValid = false;
			float m_minX;
			float m_minY;
			float m_maxX;
			float m_maxY;
			std::vector<vIndex> m_orderedX;
			std::vector<vIndex> m_orderedY;
			std::vector<Vertex> m_vertices;
			std::vector<Triangle> m_triangles;
			std::unordered_map<vIndex, std::vector<Triangle*>> m_triangleMap;

		public:
			HeightMap() = default;
			int getID() { return m_id; }
			bool withinBoundaries(float x, float y);
			int orientationTest(Vertex v1, Vertex v2, Vertex testV); // se ignora coordenada z
			bool triangleContainsPoint(Triangle t, Vertex p); // se ignora coordenada z
			int sharedVertices(Triangle t1, Triangle t2);
			void orderVerticesCCW(std::vector<vIndex>* vertices);
			void orderTriangle(Triangle* t);
			void init(std::vector<Vector3f>& vertices, std::vector<Vector3ui>& faces);
			bool isValid() { return m_isValid; }
			float getHeight(float x, float y);
			bool goesThroughTriangle(vIndex start, Vertex end, Triangle* triangle);
			Triangle* nextTriangle(Vertex start, Vertex end, Triangle* triangle);
	};

}


#endif