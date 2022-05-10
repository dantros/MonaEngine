#include "EnvironmentData.hpp"
#include <limits>
#include <algorithm>
#include "../Core/Log.hpp"
#include "../Utilities/FuncUtils.hpp"
namespace Mona{

    int MeshData::lastId = 0;

    void MeshData::init(std::vector<Vector3f>& vertices, std::vector<std::vector<unsigned int>>& faces) {
        m_id = lastId + 1;
        lastId = m_id;
        m_triangles.reserve(faces.size());
        for (int i = 0; i < faces.size(); i++) {
            Triangle t;
            int ind1 = faces[i][0];
            int ind2 = faces[i][1];
            int ind3 = faces[i][2];
            Vertex v1 = Vertex(vertices[ind1][0], vertices[ind1][1], vertices[ind1][2]);
            Vertex v2 = Vertex(vertices[ind2][0], vertices[ind2][1], vertices[ind2][2]);
            Vertex v3 = Vertex(vertices[ind3][0], vertices[ind3][1], vertices[ind3][2]);
            t.vertices = {v1, v2, v3};
        }
        m_isValid = true;

    }

    bool MeshData::withinBoundaries(float x, float y) {
        return m_minX <= x  &&  x <= m_maxX  &&  m_minY <= y  && y <= m_maxY;
    }

    int MeshData::orientationTest(Vertex v1, Vertex v2, Vertex testV){ //arista de v1 a v2, +1 si el punto esta a arriba, -1 abajo,0 si es colineal, error si v1 y v2 iguales
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
        int orientationV1V2 = orientationTest(t.vertices[0], t.vertices[1], p);
        int orientationV2V3 = orientationTest(t.vertices[1], t.vertices[2], p);
        int orientationV3V1 = orientationTest(t.vertices[2], t.vertices[0], p);
        if (orientationV1V2 + orientationV2V3 + orientationV3V1 == 3) { return true;  } //case 1: point inside triangle
        else if ((orientationV1V2 + orientationV2V3 + orientationV3V1) == 2) {
            if (orientationV1V2 == 0 || orientationV2V3 == 0 || orientationV3V1 == 0) { return true;  } //case 2: point on edge
        }
        else if (orientationV1V2 == orientationV2V3 == 0 or orientationV1V2 == orientationV3V1 == 0 or orientationV2V3 == orientationV3V1 == 0) {
            return true; //case 0: point is a vertex
        }
        else { return false; }
    }

    void MeshData::orderVerticesCCW(std::vector<Vertex>* vertices) {
        int orientation = orientationTest((*vertices)[0], (*vertices)[1], (*vertices)[2]);
        if (orientation == 1) {
            return; // ya ordenados
        }
        else if (orientation == -1) {
            std::vector<Vertex> temp = { (*vertices)[0], (*vertices)[1], (*vertices)[2] };
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
            if (connectedT != nullptr){
                if (funcUtils::findIndex<Vertex>(connectedT->vertices, t->vertices[0]) != -1 && funcUtils::findIndex<Vertex>(connectedT->vertices, t->vertices[1]) != -1) {
                    orderedTriangles[0] = connectedT;
                }
                else if (funcUtils::findIndex<Vertex>(connectedT->vertices, t->vertices[1]) != -1 && funcUtils::findIndex<Vertex>(connectedT->vertices, t->vertices[2]) != -1) {
                    orderedTriangles[1] = connectedT;
                }
                else if (funcUtils::findIndex<Vertex>(connectedT->vertices, t->vertices[2]) != -1 && funcUtils::findIndex<Vertex>(connectedT->vertices, t->vertices[0]) != -1) {
                    orderedTriangles[2] = connectedT;
                }
            }
        }                
        t->neighbors = orderedTriangles;            
        //triangles are ordered. first one shares v1 and v2 with t, second one v2 and v3, third one v3 and v1.
    }
  
        

    // debe haber pasado test de withinBoundaries
    float EnvironmentData::_getTerrainHeight(float x, float y, MeshData terrain) {
        MONA_ASSERT(terrain.withinBoundaries(x, y), "Vertex must be within boundaries of the mesh");
        return 0;
    }

}