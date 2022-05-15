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

    void EnvironmentData::addTerrain(const StaticMeshHandle& terrain) {
        InnerComponentHandle innerHandle = terrain.GetInnerHandle();
        const StaticMeshComponent* staticMesh = m_staticMeshManager->GetComponentPointer(innerHandle);
        HeightMap* heigtMap = staticMesh->GetHeightMap();
        if (heigtMap->isValid()) {
            m_terrains.push_back(innerHandle);
        }
        else {
            MONA_LOG_ERROR("Input terrain's height map was not valid");
        }
        
    }
    int EnvironmentData::removeTerrain(const StaticMeshHandle& terrain) {
        InnerComponentHandle innerHandle = terrain.GetInnerHandle();
        for (int i = 0; i < m_terrains.size(); i++) {
            if (m_terrains[i].m_generation == innerHandle.m_generation && m_terrains[i].m_index==innerHandle.m_index) {
                m_terrains.erase(m_terrains.begin() + i);
                return innerHandle.m_index;
            }
        }
        return -1;
    }

}