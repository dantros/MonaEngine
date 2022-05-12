#include "HeightMap.hpp"
#include <limits>
#include <algorithm>
#include "../Core/Log.hpp"
#include "../Utilities/FuncUtils.hpp"

namespace Mona{

    int HeightMap::lastId = 0;

    // operators
    bool operator!= (const Triangle& t1, const Triangle& t2) {
        return t1.vertices != t2.vertices;
    }
    bool operator== (const Triangle& t1, const Triangle& t2) {
        return t1.vertices == t2.vertices;
    }

    struct IndexedVertex {
        int index;
        Vertex vertex;
        static bool compareX(const IndexedVertex& v1, const IndexedVertex& v2) { return v1.vertex[0] < v2.vertex[0]; }
        static bool compareY(const IndexedVertex& v1, const IndexedVertex& v2) { return v1.vertex[1] < v2.vertex[1]; }
    };

    int HeightMap::sharedVertices(Triangle* t1, Triangle* t2) {
        int comp1 = t1->vertices[0] == t2->vertices[0] + t1->vertices[1] == t2->vertices[1] + t1->vertices[2] == t2->vertices[2];
        int comp2 = t1->vertices[0] == t2->vertices[1] + t1->vertices[1] == t2->vertices[2] + t1->vertices[2] == t2->vertices[0];
        return comp1 + comp2;
    }

    void HeightMap::init(const std::vector<Vector3f>& vertices, const std::vector<Vector3ui>& faces) {
        m_id = lastId + 1;
        lastId = m_id;
        m_vertices.reserve(vertices.size());
        m_triangles.reserve(faces.size());
        
        // guardar vertices e inicializar mapa de vertices->triangulos
        for (int i = 0; i < vertices.size(); i++) {
            Vertex v = Vertex(vertices[i][0], vertices[i][1], vertices[i][2]);
            m_vertices.push_back(v);

            m_triangleMap[i] = std::vector<Triangle*>();
        }
        // asginar vertices a triangulos
        for (int i = 0; i < faces.size(); i++) {
            Triangle t;
            vIndex ind1 = faces[i][0];
            vIndex ind2 = faces[i][1];
            vIndex ind3 = faces[i][2];
            t.vertices = { ind1, ind2, ind3 };
            m_triangles.push_back(t);

            m_triangleMap[ind1].push_back(&t);
            m_triangleMap[ind2].push_back(&t);
            m_triangleMap[ind3].push_back(&t);
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
                if (sharedVertices(&trI, &trJ) == 2) {
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

    bool HeightMap::withinBoundaries(float x, float y) {
        return m_minX <= x && x <= m_maxX && m_minY <= y && y <= m_maxY;
    }

    int HeightMap::orientationTest(const Vertex& v1, const Vertex& v2, const Vertex& testV) { //arista de v1 a v2, +1 si el punto esta a arriba, -1 abajo,0 si es colineal, error(-2) si v1 y v2 iguales
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
            return -2;
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

    bool HeightMap::triangleContainsPoint(Triangle* t, const Vertex& p) {
        int orientationV1V2 = orientationTest(m_vertices[t->vertices[0]], m_vertices[t->vertices[1]], p);
        int orientationV2V3 = orientationTest(m_vertices[t->vertices[1]], m_vertices[t->vertices[2]], p);
        int orientationV3V1 = orientationTest(m_vertices[t->vertices[2]], m_vertices[t->vertices[0]], p);
        if (orientationV1V2 + orientationV2V3 + orientationV3V1 == 3) { return true; } //case 1: point inside triangle
        else if ((orientationV1V2 + orientationV2V3 + orientationV3V1) == 2) {
            if (orientationV1V2 == 0 || orientationV2V3 == 0 || orientationV3V1 == 0) { return true; } //case 2: point on edge
        }
        else if (orientationV1V2 == orientationV2V3 == 0 or orientationV1V2 == orientationV3V1 == 0 or orientationV2V3 == orientationV3V1 == 0) {
            return true; //case 0: point is a vertex
        }
        return false;
    }

    void HeightMap::orderVerticesCCW(std::vector<vIndex>* vertices) {
        int orientation = orientationTest(m_vertices[(*vertices)[0]], m_vertices[(*vertices)[1]], m_vertices[(*vertices)[2]]);
        if (orientation == 1) {
            return; // ya ordenados
        }
        else if (orientation == -1) {
            std::vector<vIndex> temp = { (*vertices)[0], (*vertices)[1], (*vertices)[2] };
            (*vertices)[0] = temp[1];
            (*vertices)[1] = temp[0];
            (*vertices)[2] = temp[2];
            return;
        }
        else {
            MONA_LOG_ERROR("Not a triangle, or projection of vertical triangle");
        }
    }

    void HeightMap::orderTriangle(Triangle* t) {
        orderVerticesCCW(&(t->vertices));
        std::vector<Triangle*> orderedTriangles = { nullptr, nullptr, nullptr };
        for (int i = 0; i < 3; i++) {
            Triangle* connectedT = t->neighbors[i];
            if (connectedT != nullptr) {
                if (funcUtils::findIndex<vIndex>(connectedT->vertices, t->vertices[0]) != -1 && funcUtils::findIndex<vIndex>(connectedT->vertices, t->vertices[1]) != -1) {
                    orderedTriangles[0] = connectedT;
                }
                else if (funcUtils::findIndex<vIndex>(connectedT->vertices, t->vertices[1]) != -1 && funcUtils::findIndex<vIndex>(connectedT->vertices, t->vertices[2]) != -1) {
                    orderedTriangles[1] = connectedT;
                }
                else if (funcUtils::findIndex<vIndex>(connectedT->vertices, t->vertices[2]) != -1 && funcUtils::findIndex<vIndex>(connectedT->vertices, t->vertices[0]) != -1) {
                    orderedTriangles[2] = connectedT;
                }
            }
        }
        t->neighbors = orderedTriangles;
        //triangles are ordered. first one shares v1 and v2 with t, second one v2 and v3, third one v3 and v1.
    }

    bool HeightMap::goesThroughTriangle(vIndex start, Vertex end, Triangle* triangle) { //checks if line (starting in vertex of triangle) goes through triangle(or coincides with edge) or passes outside
        int initialVNum;
        if (start == triangle->vertices[0]) { initialVNum = 0; }
        else if (start == triangle->vertices[1]) { initialVNum = 1; }
        else if (start == triangle->vertices[2]){ initialVNum = 2; }
        else {
            MONA_LOG_ERROR("Starting vertex not in triangle");
            return false;
        }
        int orientation1 = orientationTest(m_vertices[start], end, m_vertices[triangle->vertices[(initialVNum + 1) % 3]]);
        int orientation2 = orientationTest(m_vertices[start], end, m_vertices[triangle->vertices[(initialVNum + 2) % 3]]);
        if (orientation1 == -1 && orientation2 == 1) { return true; } // passes inside
        else if (orientation1 == 0 && orientation2 == 1) { return true; } //coincides with edge v1v2
        else if (orientation2 == 0 && orientation1 == -1) { return true; } // coincides with edge v3v1
        return false; // passes outside
    }

    Triangle* HeightMap::nextTriangle(Vertex start, Vertex end, Triangle* triangle) { // through which triangle does the line continue
        int orientationV1 = orientationTest(start, end, m_vertices[triangle->vertices[0]]);
        int orientationV2 = orientationTest(start, end, m_vertices[triangle->vertices[1]]);
        int orientationV3 = orientationTest(start, end, m_vertices[triangle->vertices[2]]);
        std::vector<int> orientations = { orientationV1, orientationV2, orientationV3 };
        // line goes through an edge
        for (int v = 0; v < 3; v++) {
            int nextV_first = (v + 1) % 3;
            int nextV_second = (v + 2) % 3;
            // enters through t associated with nextV_first
            if (orientations[v] == -1 && orientations[nextV_first] == 1 && orientations[nextV_second] == -1) {
                return triangle->neighbors[v];
            }
            // enters through t associated with nextV_second
            if (orientations[v] == -1 && orientations[nextV_first] == 1 && orientations[nextV_second] == 1) {
                return triangle->neighbors[v];
            }
            // enters through nextV_second
            if (orientations[v] == -1 && orientations[nextV_first] == 1 && orientations[nextV_second] == 0) {
                return triangle->neighbors[v];
            }
        }
        // chech if line goes through vertex
        int outVNum = -1;
        //  coincides with edge
        for (int v = 0; v < 3; v++) {
            int nextV_first = (v + 1) % 3;
            int nextV_second = (v + 2) % 3;
            if (orientations[v] == 0 && orientations[nextV_first] == 0 && orientations[nextV_second] == 1) {
                outVNum = nextV_first;
                break;
            }
        }
        //  does not coincide with edge
        if (outVNum == -1) {
            for (int v = 0; v < 3; v++) {
                if (orientations[v] == 0) {
                    outVNum = v;
                    break;
                }
            }
        }
        
        if (outVNum != -1) {
            vIndex outV = triangle->vertices[outVNum];
            for (int i = 0; i < m_triangleMap[outV].size(); i++) {
                Triangle* currT = m_triangleMap[outV][i];
                if (currT != triangle && goesThroughTriangle(outV, end, currT)) {
                    return currT;
                }
            }
        }
        return nullptr; // la linea sale de la malla
        
        
    }

    vIndex HeightMap::findCloseVertex(float x, float y) {
        std::vector<vIndex> ansXY;
        std::vector<std::vector<vIndex>*> arrays = { &m_orderedX, &m_orderedY };

        for (int i = 0; i < 2; i++) {
            int subArraySize = m_vertices.size();
            int foundIndex = m_vertices.size() - 1; // parte al final del subarreglo
            while (subArraySize > 1) {
                bool balance = subArraySize % 2 != 0;
                subArraySize = subArraySize / 2;

                foundIndex = foundIndex - subArraySize + !balance;
                if (x < m_vertices[(*arrays[i])[foundIndex]][0]) {
                    foundIndex -= 1;
                }
                else {
                    foundIndex += subArraySize;
                    subArraySize += balance;
                }
            }
            ansXY.push_back(foundIndex);
        }
        if (m_orderedX[ansXY[0]] == m_orderedY[ansXY[1]]) { return m_orderedX[ansXY[0]]; }
        int startX = ansXY[0];
        int endX = ansXY[0];
        int startY = ansXY[1];
        int endY = ansXY[1];
        // Expand both subarrays gradually trying to find a coincidence in vertex indexes.
        while (startX > 0 || endX < m_vertices.size()-1 || startY > 0 || endY < m_vertices.size()-1) {
            if (startX > 0) { 
                startX -= 1;
                if (funcUtils::findIndexSubArray(m_orderedY, m_orderedX[startX], startY, endY) != -1) { 
                    return m_orderedX[startX]; 
                }
            }
            if (endX < m_vertices.size()-1) { 
                endX += 1;
                if (funcUtils::findIndexSubArray(m_orderedY, m_orderedX[endX], startY, endY) != -1) {
                    return m_orderedX[startX];
                }
            }
            if (startY > 0) { 
                startY -= 1;
                if (funcUtils::findIndexSubArray(m_orderedX, m_orderedY[startY], startX, endX) != -1) {
                    return m_orderedY[startY];
                }
            }
            if (endY < m_vertices.size()-1) { 
                endY += 1;
                if (funcUtils::findIndexSubArray(m_orderedX, m_orderedY[endY], startX, endX) != -1) {
                    return m_orderedY[startY];
                }
            }        
        }

        return -1;
    }

    Vertex interpolateVertex(Vertex v1, Vertex v2, float fraction) {
        float deltaX = (v2[0] - v1[0]) * fraction;
        float deltaY = (v2[1] - v1[1]) * fraction;
        float deltaZ = (v2[2] - v1[2]) * fraction;

        return Vertex(v1[0] + deltaX, v2[1] + deltaY, v2[2] + deltaZ);
    }

    float HeightMap::getInterpolatedHeight(Triangle* t, float x, float y) {
        Vertex v1 = m_vertices[t->vertices[0]];
        Vertex v2 = m_vertices[t->vertices[1]];
        Vertex v3 = m_vertices[t->vertices[2]];
        float minX = std::min(std::min(v1[0], v2[0]), v3[0]);
        float maxX = std::max(std::max(v1[0], v2[0]), v3[0]);
        if (minX <= x && x <= maxX) {
            MONA_LOG_ERROR("Target point not inside found triangle.");
            return std::numeric_limits<float>::min();
        }
        std::vector vertices = { v1, v2, v3 };
        Vertex targetPoint = Vertex(x, y, 0);
        //  check if points coincides with edge or vertex
        for (int i = 0; i < 3; i++) {
            if (orientationTest(vertices[i], vertices[(i + 1) % 3], targetPoint) == 0) {
                float frac = (x - vertices[i][0]) / (vertices[(i + 1) % 3][0] - vertices[i][0]);
                return interpolateVertex(vertices[i], vertices[(i + 1) % 3], frac)[2];
            }
        }
        // find edges to project point onto
        std::vector<std::pair<Vertex, Vertex>> edges;
        for (int i = 0; i < 3; i++) {
            if (vertices[i][0] <= x && x <= vertices[(i + 1) % 3][0]) {
                edges.push_back(std::pair<Vertex, Vertex>(vertices[i], vertices[(i + 1) % 3]));
            }
        }
        if (edges.size() != 2) {
            MONA_LOG_ERROR("Should have found two edges");
            return std::numeric_limits<float>::min();
        }
        
        float frac1 = (x - edges[0].first[0]) / (edges[0].second[0] - edges[0].first[0]);
        float frac2 = (x - edges[1].first[0]) / (edges[1].second[0] - edges[1].first[0]);
        Vertex interpolated1 = interpolateVertex(edges[0].first, edges[0].second, frac1);
        Vertex interpolated2 = interpolateVertex(edges[1].first, edges[1].second, frac2);

        float fracFinal = (y - interpolated1[1]) / (interpolated2[1] - interpolated1[1]);
        return interpolateVertex(interpolated1, interpolated2, fracFinal)[2];
    }

    float HeightMap::getHeight(float x, float y) {
        if (!withinBoundaries(x, y)) {
            MONA_LOG_WARNING("Point is out of bounds");
            return std::numeric_limits<float>::min();
        }
        // encontrar vertice cercano
        vIndex closeV = findCloseVertex(x, y);
        if (closeV == -1) {
            MONA_LOG_ERROR("Could not find close vertex");
            return std::numeric_limits<float>::min();
        }

        // encontrar triangulo contenedor
        Triangle* foundT = nullptr;
        Triangle* currentT = m_triangleMap[closeV][0];
        Vertex targetPoint = Vertex(x, y, 0);
        while (!triangleContainsPoint(currentT, targetPoint)) {
            Triangle* nextT = nextTriangle(m_vertices[closeV], targetPoint, currentT);
            if (nextT == nullptr) {
                MONA_LOG_WARNING("Point is out of bounds");
                return std::numeric_limits<float>::min();
            }
            currentT = nextT;
        }
        foundT = currentT;
        // interpolar altura
        return getInterpolatedHeight(foundT, x, y);


    }

}