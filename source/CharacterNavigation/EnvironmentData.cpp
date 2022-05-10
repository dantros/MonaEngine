#include "EnvironmentData.hpp"
#include <limits>
#include <algorithm>
#include "../Core/Log.hpp"
namespace Mona{

    struct IndexedVertex {
        int index;
        Vector3f vertex;
        static bool compareX(IndexedVertex& v1, IndexedVertex& v2) { return v1.vertex[0] < v2.vertex[0]; }
        static bool compareY(IndexedVertex& v1, IndexedVertex& v2) { return v1.vertex[1] < v2.vertex[1]; }
    };
    int MeshData::lastId = 0;

    void MeshData::init(std::vector<Vector3f>& vertices, std::vector<Triangle>& faces) {
        m_id = lastId + 1;
        lastId = m_id;
        m_vertices = vertices;
        std::vector<IndexedVertex> indexedArr;
        indexedArr.reserve(vertices.size());
        m_orderedX.reserve(vertices.size());
        m_orderedY.reserve(vertices.size());
        for (int i = 0; i < vertices.size(); i++) {
            indexedArr.push_back(IndexedVertex(i, vertices[i]));
            m_triangleMap[i] = std::vector<Triangle*>();
        }
        std::sort(indexedArr.begin(), indexedArr.end(), IndexedVertex::compareX);
        for (int i = 0; i < indexedArr.size(); i++) {
            m_orderedX.push_back(indexedArr[i].index);
        }
        std::sort(indexedArr.begin(), indexedArr.end(), IndexedVertex::compareY);
        for (int i = 0; i < indexedArr.size(); i++) {
            m_orderedY.push_back(indexedArr[i].index);
        }

        m_minX = std::numeric_limits<float>::max();
        m_minY = std::numeric_limits<float>::max();
        m_maxX = std::numeric_limits<float>::min();
        m_maxY = std::numeric_limits<float>::min();

        // guardar valores minimos y maximos
        for (int i = 0; i < vertices.size(); i++) {

            int x = vertices[i][0];
            int y = vertices[i][1];
            if (x < m_minX) { m_minX = x; }
            if (y < m_minY) { m_minY = y; }
            if (x > m_maxX) { m_maxX = x; }
            if (y > m_maxY) { m_maxY = y; }

        }

        m_triangles = faces;
        for (int i = 0; i < m_triangles.size(); i++) {
            Triangle triangle = m_triangles[i];
            m_triangleMap[triangle[0]].push_back(&triangle);
            m_triangleMap[triangle[1]].push_back(&triangle);
            m_triangleMap[triangle[2]].push_back(&triangle);
        }

        m_isValid = true;

    }

    bool MeshData::withinBoundaries(float x, float y) {
        return m_minX <= x  &&  x <= m_maxX  &&  m_minY <= y  && y <= m_maxY;
    }

    int MeshData::orientationTest(Vector3f v1, Vector3f v2, Vector3f testV){ //arista de v1 a v2, +1 si el punto esta a arriba, -1 abajo,0 si es colineal, error si v1 y v2 iguales
        float x1 = v1[0];
        float y1 = v1[1];
        float x2 = v2[0];
        float y2 = v2[1];

        int orientationV1V2 = 1;
        if (x1 > x2) {
            orientationV1V2 = -1;
        }
            
        if (testV[0] == v1[0] && testV[1] == v1[1] || testV[0] == v2[0] && testV[1] == v2[1]) {
            return 0;
        }
                
        if (x1 == x2 and y1 == y2) {
            MONA_LOG_ERROR("vertices are equal!");
        }
        if (x1 == x2) {
            if (y1 > y2) { orientationV1V2 = -1; }
            if (testV[0] == x1) { return 0; }
            else if (testV[0] > x1) { return -1 * orientationV1V2; }     
            else { return 1 * orientationV1V2; }
        }
            
    
        float yOnLine = ((y2 - y1) / (x2 - x1)) * (testV[0] - x1) + y1;
        if (testV[1] > yOnLine) { return 1 * orientationV1V2; } 
        else if (testV[1] < yOnLine) { return -1 * orientationV1V2; }
        else { return 0; }
    }

    bool MeshData::triangleContainsPoint(Triangle t, Vector3f p) {
        int orientationV1V2 = orientationTest(m_vertices[t[0]], m_vertices[t[1]], p);
        int orientationV2V3 = orientationTest(m_vertices[t[1]], m_vertices[t[2]], p);
        int orientationV3V1 = orientationTest(m_vertices[t[2]], m_vertices[t[0]], p);
        if (orientationV1V2 + orientationV2V3 + orientationV3V1 == 3) { return true;  } //case 1: point inside triangle
        else if ((orientationV1V2 + orientationV2V3 + orientationV3V1) == 2) {
            if (orientationV1V2 == 0 || orientationV2V3 == 0 || orientationV3V1 == 0) { return true;  } //case 2: point on edge
        }
        else if (orientationV1V2 == orientationV2V3 == 0 or orientationV1V2 == orientationV3V1 == 0 or orientationV2V3 == orientationV3V1 == 0) {
            return true; //case 0: point is a vertex
        }
        else { return false; }
    }

    // debe haber pasado test de withinBoundaries
    float EnvironmentData::_getTerrainHeight(float x, float y, MeshData terrain) {
        MONA_ASSERT(terrain.withinBoundaries(x, y), "Vertex must be within boundaries of the mesh");
        float xStartFrac = (x - terrain.m_minX) / (terrain.m_maxX - terrain.m_minX);
        int xStartInd = terrain.m_vertices.size() * xStartFrac;
        return 0;
    }

}