#include "EnvironmentData.hpp"
#include <limits>
#include <algorithm>
#include "../Core/Log.hpp"
#include "../Utilities/FuncUtils.hpp"
namespace Mona{        

    float EnvironmentData::getTerrainHeight(float x, float y) {
        float maxHeight = std::numeric_limits<float>::min();
        for (int i = 0; i < m_terrains.size(); i++) {
            float height = m_terrains[i]->getHeight(x, y);
            if (height > maxHeight) { maxHeight = height; }
        }
        return maxHeight;
    }

    void EnvironmentData::addTerrain(HeightMap* terrain) {
        if (terrain->isValid()) {
            m_terrains.push_back(terrain);
        }
        else {
            MONA_LOG_ERROR("Input terrain was not valid");
        }
        
    }
    int EnvironmentData::removeTerrain(HeightMap* terrain) {
        for (int i = 0; i < m_terrains.size(); i++) {
            if (m_terrains[i]->getID() == terrain->getID()) {
                m_terrains.erase(m_terrains.begin() + i);
                return m_terrains[i]->getID();
            }
        }
        return -1;
    }

}