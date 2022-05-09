#include "EnvironmentData.hpp"
#if 0
namespace Mona{

    void EnvironmentData::addTerrain(StaticMeshHandle& terrainHandle) {
        auto inputGO = m_world.GetOwner(terrainHandle).GetInnerHandle();
        for (int i = 0; i < m_terrainHandles.size(); i++) {
            auto gameObject = m_world.GetOwner(m_terrainHandles[i]).GetInnerHandle();
            if (inputGO.m_generation==gameObject.m_generation && inputGO.m_index==gameObject.m_index){
                return;
            }
        }
        m_terrainHandles.push_back(terrainHandle);
    }

    int EnvironmentData::removeTerrain(StaticMeshHandle& terrainHandle){
        auto inputGO = m_world.GetOwner(terrainHandle).GetInnerHandle();
        for (int i = 0; i < m_terrainHandles.size(); i++) {
            auto gameObject = m_world.GetOwner(m_terrainHandles[i]).GetInnerHandle();
            if (inputGO.m_generation==gameObject.m_generation && inputGO.m_index==gameObject.m_index){
                m_terrainHandles.erase(m_terrainHandles.begin()+i);
                return i;
            }
        }
        return -1;
    }

    float EnvironmentData::terrainHeight(StaticMeshHandle terrainHandle){
        auto mesh = terrainHandle;
    }

}
#endif