#include "HeightMap.hpp"
#include <limits>
#include <algorithm>
#include "../Core/Log.hpp"
#include "../Core/FuncUtils.hpp"

namespace Mona{

    HeightMap::HeightMap(const glm::vec2& bottomLeft, const glm::vec2& topRight, float (*heightFunc)(float, float)) {
        m_minX = bottomLeft[0];
		m_minY = bottomLeft[1];
		m_maxX = topRight[0];
        m_maxY = topRight[1];
		m_heightFunc = heightFunc;
        m_isValid = true;
    }

    bool HeightMap::withinBoundaries(float x, float y) {
        return m_minX <= x && x <= m_maxX && m_minY <= y && y <= m_maxY;
    }

    float HeightMap::getHeight(float x, float y) {
        if (!withinBoundaries(x, y)) {
            MONA_LOG_WARNING("HeightMap: Point is out of bounds");
            return std::numeric_limits<float>::lowest();
        }

        return m_heightFunc(x, y);
    }

}