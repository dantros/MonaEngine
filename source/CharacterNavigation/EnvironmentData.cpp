#include "EnvironmentData.hpp"
#include <limits>
#include <algorithm>
#include "../Core/Log.hpp"
#include "../Utilities/FuncUtils.hpp"
#include <glm/glm.hpp>
namespace Mona{        

    EnvironmentData::EnvironmentData(ComponentManager<TransformComponent>* transformManager, ComponentManager<StaticMeshComponent>* staticMeshManager) {
        m_transformManager = transformManager;
        m_staticMeshManager = staticMeshManager;
    }
    float EnvironmentData::getTerrainHeight(float x, float y) {
        float maxHeight = std::numeric_limits<float>::min();
        for (int i = 0; i < m_terrains.size(); i++) {
            if (!m_staticMeshManager->IsValid(m_terrains[i])) {
                m_terrains.erase(m_terrains.begin() + i);
                MONA_LOG_WARNING("Saved terrain was not valid, so it was removed.");
                continue;
            }
            const StaticMeshComponent* staticMesh = m_staticMeshManager->GetComponentPointer(m_terrains[i]);
            HeightMap* heigtMap = staticMesh->GetHeightMap();
            GameObject* staticMeshOwner = m_staticMeshManager->GetOwner(m_terrains[i]);
            TransformComponent* staticMeshTransform = m_transformManager->GetComponentPointer(staticMeshOwner->GetInnerComponentHandle<TransformComponent>());

            // trasladar punto a espacio local de la malla
            glm::vec4 basePoint = { x, y, 0, 1};
            glm::vec4 meshPoint = glm::inverse(staticMeshTransform->GetModelMatrix())*basePoint;

            float localHeight = heigtMap->getHeight(meshPoint[0], meshPoint[1]);
            glm::vec4 localResult = { meshPoint[0], meshPoint[1], localHeight, 1 };
            // volver a espacio global
            glm::vec4 globalResult = staticMeshTransform->GetModelMatrix() * localResult;
            if (globalResult[2] > maxHeight) { maxHeight = globalResult[2]; }
        }
        return maxHeight;
    }

    void EnvironmentData::addTerrain(const StaticMeshHandle& terrain) {
        InnerComponentHandle innerHandle = terrain.GetInnerHandle();
        const StaticMeshComponent* staticMesh = m_staticMeshManager->GetComponentPointer(innerHandle);
        HeightMap* heigtMap = staticMesh->GetHeightMap();
        if (heigtMap->isValid() && m_staticMeshManager->IsValid(innerHandle)) {
            m_terrains.push_back(innerHandle);
        }
        else {
            MONA_LOG_ERROR("Input terrain was not valid");
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