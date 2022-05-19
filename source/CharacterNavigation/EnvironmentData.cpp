#include "EnvironmentData.hpp"
#include <limits>
#include <algorithm>
#include "../Core/Log.hpp"
#include "../Utilities/FuncUtils.hpp"
#include <glm/glm.hpp>
namespace Mona{        

    Terrain::Terrain(const GameObject& staticMeshObject) {
        MONA_ASSERT(staticMeshObject.HasComponent<StaticMeshComponent>(), "Terrain object must have a satitc mesh component");
        m_transformHandle = staticMeshObject.GetInnerComponentHandle<TransformComponent>();
        m_meshHandle = staticMeshObject.GetInnerComponentHandle<StaticMeshComponent>();
    }

    EnvironmentData::EnvironmentData(ComponentManager<TransformComponent>* transformManager, ComponentManager<StaticMeshComponent>* staticMeshManager) {
        m_transformManager = transformManager;
        m_staticMeshManager = staticMeshManager;
    }
    float EnvironmentData::getTerrainHeight(float x, float y) {
        float maxHeight = std::numeric_limits<float>::min();
        for (int i = 0; i < m_terrains.size(); i++) {
            if (!m_staticMeshManager->IsValid(m_terrains[i].m_meshHandle)) {
                m_terrains.erase(m_terrains.begin() + i);
                MONA_LOG_WARNING("Saved terrain was not valid, so it was removed.");
                continue;
            }
            const StaticMeshComponent* staticMesh = m_staticMeshManager->GetComponentPointer(m_terrains[i].m_meshHandle);
            HeightMap* heigtMap = staticMesh->GetHeightMap();
            TransformComponent* staticMeshTransform = m_transformManager->GetComponentPointer(m_terrains[i].m_transformHandle);

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

    void EnvironmentData::addTerrain(const Terrain& terrain) {
        InnerComponentHandle meshInnerHandle = terrain.m_meshHandle;
        const StaticMeshComponent* staticMesh = m_staticMeshManager->GetComponentPointer(meshInnerHandle);
        HeightMap* heigtMap = staticMesh->GetHeightMap();
        if (heigtMap->isValid() && m_staticMeshManager->IsValid(meshInnerHandle)) {
            m_terrains.push_back(terrain);
        }
        else {
            MONA_LOG_ERROR("Input terrain was not valid");
        }        
    }
    int EnvironmentData::removeTerrain(const Terrain& terrain) {
        InnerComponentHandle meshInnerHandle = terrain.m_meshHandle;
        for (int i = 0; i < m_terrains.size(); i++) {
            if (m_terrains[i].m_meshHandle.m_generation == meshInnerHandle.m_generation && m_terrains[i].m_meshHandle.m_index==meshInnerHandle.m_index) {
                m_terrains.erase(m_terrains.begin() + i);
                return meshInnerHandle.m_index;
            }
        }
        return -1;
    }

}