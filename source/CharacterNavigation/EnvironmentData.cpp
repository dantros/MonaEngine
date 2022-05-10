#include "EnvironmentData.hpp"
#include <limits>
#include <algorithm>
namespace Mona{

    VertexData::VertexData(std::vector<Vector3f> vArray) {
        m_id = lastId + 1;
        lastId = m_id;
        m_vertexArray = vArray;
        std::vector<IndexedVertex> indexedArr;
        for (int i = 0; i < vArray.size(); i++) {
            indexedArr.push_back(IndexedVertex(i, vArray[i]));
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
        for (int i = 0; i < vArray.size(); i++) {
            
            int x = vArray[i][0];
            int y = vArray[i][1];
            if (x < m_minX) { m_minX = x; }
            if (y < m_minY) { m_minY = y; }
            if (x > m_maxX) { m_maxX = x; }
            if (y > m_maxY) { m_maxY = y; }

        }       
    }

}