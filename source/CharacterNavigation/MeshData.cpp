#include "MeshData.hpp"
#include <limits>
#include <algorithm>
#include "../Core/Log.hpp"
#include "../Utilities/FuncUtils.hpp"

namespace Mona{

    int MeshData::lastId = 0;

    // operators
    bool operator!= (const Vertex& v1, const Vertex& v2) {
        return v1.x != v2.x || v1.y != v2.y || v1.z != v2.z;
    }
    bool operator== (const Vertex& v1, const Vertex& v2) {
        return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
    }
    bool operator!= (const Triangle& t1, const Triangle& t2) {
        return t1.vertices != t2.vertices;
    }
    bool operator== (const Triangle& t1, const Triangle& t2) {
        return t1.vertices == t2.vertices;
    }

    struct IndexedVertex {
        int index;
        Vertex vertex;
        static bool compareX(IndexedVertex& v1, IndexedVertex& v2) { return v1.vertex.x < v2.vertex.x; }
        static bool compareY(IndexedVertex& v1, IndexedVertex& v2) { return v1.vertex.y < v2.vertex.y; }
    };

    int MeshData::sharedVertices(Triangle t1, Triangle t2) {
        int comp1 = t1.vertices[0] == t2.vertices[0] + t1.vertices[1] == t2.vertices[1] + t1.vertices[2] == t2.vertices[2];
        int comp2 = t1.vertices[0] == t2.vertices[1] + t1.vertices[1] == t2.vertices[2] + t1.vertices[2] == t2.vertices[0];
        return comp1 + comp2;
    }

    void MeshData::init(std::vector<Vector3f>& vertices, std::vector<Vector3ui>& faces) {
        m_id = lastId + 1;
        lastId = m_id;
        m_vertices.reserve(vertices.size());
        m_triangles.reserve(faces.size());
        
        // guardar vertices
        for (int i = 0; i < vertices.size(); i++) {
            Vertex v = Vertex(vertices[i][0], vertices[i][1], vertices[i][2]);
            m_vertices.push_back(v);
        }
        // asginar vertices a triangulos
        for (int i = 0; i < faces.size(); i++) {
            Triangle t;
            Index ind1 = faces[i][0];
            Index ind2 = faces[i][1];
            Index ind3 = faces[i][2];
            t.vertices = { ind1, ind2, ind3 };
            m_triangles.push_back(t);
        }

        // vincular triangulos con sus vecinos
        for (int i = 0; i < m_triangles.size(); i++) {
            Triangle trI = m_triangles[i];
            for (int j = i+1; j < m_triangles.size(); j++){ // se comienza desde i+1 porque los anteriores ya fueron vinculados (vinculos bilaterales)
                Triangle trJ = m_triangles[j];
                if (trI.neighbors[0] != nullptr && trI.neighbors[1] != nullptr && trI.neighbors[2] != nullptr) {
                    break; // todos los vecinos encontrados
                }
                if (funcUtils::findIndex(trI.neighbors, &trJ) != -1) {
                    continue; // ya esta trJ en los vecinos de trI
                }
                if (sharedVertices(trI, trJ) == 2) {
                    for (int k = 0; k < 3; k++) {
                        if (trI.neighbors[k] == nullptr) { // se asigna al primer lugar vacio
                            trI.neighbors[k] = &trJ;
                            break;
                        }
                    }
                    for (int k = 0; k < 3; k++) {
                        if (trJ.neighbors[k] == nullptr) { // se asigna al primer lugar vacio
                            trJ.neighbors[k] = &trI;
                            break;
                        }
                    }
                }
            }
        }

        // ordenamos los vertices dentro de los triangulos
        for (int i = 0; i < m_triangles.size(); i++) {
            orderTriangle(&m_triangles[i]);
        }

        std::vector<IndexedVertex> indexedArr;
        indexedArr.reserve(vertices.size());
        m_orderedX.reserve(vertices.size());
        m_orderedY.reserve(vertices.size());
        // generamos arreglos que ordenan los vertices segun su valor en X y en Y
        std::sort(indexedArr.begin(), indexedArr.end(), IndexedVertex::compareX);
        for (int i = 0; i < indexedArr.size(); i++) {
            m_orderedX.push_back(indexedArr[i].index);
        }
        std::sort(indexedArr.begin(), indexedArr.end(), IndexedVertex::compareY);
        for (int i = 0; i < indexedArr.size(); i++) {
            m_orderedY.push_back(indexedArr[i].index);
        }
        m_isValid = true;

    }

    bool MeshData::withinBoundaries(float x, float y) {
        return m_minX <= x && x <= m_maxX && m_minY <= y && y <= m_maxY;
    }

    int MeshData::orientationTest(Vertex v1, Vertex v2, Vertex testV) { //arista de v1 a v2, +1 si el punto esta a arriba, -1 abajo,0 si es colineal, error si v1 y v2 iguales
        float x1 = v1.x;
        float y1 = v1.y;
        float x2 = v2.x;
        float y2 = v2.y;

        int orientationV1V2 = 1;
        if (x1 > x2) {
            orientationV1V2 = -1;
        }

        if (testV.x == v1.x && testV.y == v1.y || testV.x == v2.x && testV.y == v2.y) {
            return 0;
        }

        if (x1 == x2 and y1 == y2) {
            MONA_LOG_ERROR("vertices are equal!");
        }
        if (x1 == x2) {
            if (y1 > y2) { orientationV1V2 = -1; }
            if (testV.x == x1) { return 0; }
            else if (testV.x > x1) { return -1 * orientationV1V2; }
            else { return 1 * orientationV1V2; }
        }


        float yOnLine = ((y2 - y1) / (x2 - x1)) * (testV.x - x1) + y1;
        if (testV.y > yOnLine) { return 1 * orientationV1V2; }
        else if (testV.y < yOnLine) { return -1 * orientationV1V2; }
        else { return 0; }
    }

    bool MeshData::triangleContainsPoint(Triangle t, Vertex p) {
        int orientationV1V2 = orientationTest(m_vertices[t.vertices[0]], m_vertices[t.vertices[1]], p);
        int orientationV2V3 = orientationTest(m_vertices[t.vertices[1]], m_vertices[t.vertices[2]], p);
        int orientationV3V1 = orientationTest(m_vertices[t.vertices[2]], m_vertices[t.vertices[0]], p);
        if (orientationV1V2 + orientationV2V3 + orientationV3V1 == 3) { return true; } //case 1: point inside triangle
        else if ((orientationV1V2 + orientationV2V3 + orientationV3V1) == 2) {
            if (orientationV1V2 == 0 || orientationV2V3 == 0 || orientationV3V1 == 0) { return true; } //case 2: point on edge
        }
        else if (orientationV1V2 == orientationV2V3 == 0 or orientationV1V2 == orientationV3V1 == 0 or orientationV2V3 == orientationV3V1 == 0) {
            return true; //case 0: point is a vertex
        }
        return false;
    }

    void MeshData::orderVerticesCCW(std::vector<Index>* vertices) {
        int orientation = orientationTest(m_vertices[(*vertices)[0]], m_vertices[(*vertices)[1]], m_vertices[(*vertices)[2]]);
        if (orientation == 1) {
            return; // ya ordenados
        }
        else if (orientation == -1) {
            std::vector<Index> temp = { (*vertices)[0], (*vertices)[1], (*vertices)[2] };
            (*vertices)[0] = temp[1];
            (*vertices)[1] = temp[0];
            (*vertices)[2] = temp[2];
            return;
        }
        else {
            MONA_LOG_ERROR("Not a triangle");
        }
    }

    void MeshData::orderTriangle(Triangle* t) {
        orderVerticesCCW(&(t->vertices));
        std::vector<Triangle*> orderedTriangles = { nullptr, nullptr, nullptr };
        for (int i = 0; i < 3; i++) {
            Triangle* connectedT = t->neighbors[i];
            if (connectedT != nullptr) {
                if (funcUtils::findIndex<Index>(connectedT->vertices, t->vertices[0]) != -1 && funcUtils::findIndex<Index>(connectedT->vertices, t->vertices[1]) != -1) {
                    orderedTriangles[0] = connectedT;
                }
                else if (funcUtils::findIndex<Index>(connectedT->vertices, t->vertices[1]) != -1 && funcUtils::findIndex<Index>(connectedT->vertices, t->vertices[2]) != -1) {
                    orderedTriangles[1] = connectedT;
                }
                else if (funcUtils::findIndex<Index>(connectedT->vertices, t->vertices[2]) != -1 && funcUtils::findIndex<Index>(connectedT->vertices, t->vertices[0]) != -1) {
                    orderedTriangles[2] = connectedT;
                }
            }
        }
        t->neighbors = orderedTriangles;
        //triangles are ordered. first one shares v1 and v2 with t, second one v2 and v3, third one v3 and v1.
    }

    int MeshData::goesThroughTriangle(Vertex v1, Vertex v2, Triangle triangle) { //checks if line (starting in vertex of triangle) goes through triangle (1), coincides with edge(0) or passes outside(-1)
        int initialVIndex;
        if (v1 == m_vertices[triangle.vertices[0]]) { initialVIndex = 0; }
        else if (v1 == m_vertices[triangle.vertices[1]]) { initialVIndex = 1; }
        else if (v1 == m_vertices[triangle.vertices[2]]){ initialVIndex = 2; }
        else {
            MONA_LOG_ERROR("Starting vertex not in triangle");
            return -2;
        }
        int orientation1 = orientationTest(v1, v2, m_vertices[triangle.vertices[(initialVIndex + 1) % 3]]);
        int orientation2 = orientationTest(v1, v2, m_vertices[triangle.vertices[(initialVIndex + 2) % 3]]);
        if (orientation1 == -1 && orientation2 == 1) { return 1; }
        else if (orientation1 == 0 && orientation2 == 1 || orientation2 == 0 && orientation1 == -1) { return 0; }
        return -1;
    }

}