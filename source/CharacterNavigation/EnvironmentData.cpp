#include "EnvironmentData.hpp"
#include "../World/ComponentManager.hpp"
#include <limits>
#include <algorithm>
#include "../Core/Log.hpp"
#include "../Utilities/FuncUtils.hpp"
#include <glm/glm.hpp>
namespace Mona{        

    Terrain::Terrain(const GameObject& staticMeshObject) {
        MONA_ASSERT(staticMeshObject.HasComponent<StaticMeshComponent>(), "EnvironmentData: Terrain object must have a satitc mesh component");
        m_transformHandle = staticMeshObject.GetInnerComponentHandle<TransformComponent>();
        m_meshHandle = staticMeshObject.GetInnerComponentHandle<StaticMeshComponent>();
    }

    void EnvironmentData::validateTerrains(ComponentManager<StaticMeshComponent>& staticMeshManager) {
        for (int i = 0; i < m_terrains.size(); i++) {
            InnerComponentHandle meshInnerHandle = m_terrains[i].m_meshHandle;
            const StaticMeshComponent* staticMesh = staticMeshManager.GetComponentPointer(meshInnerHandle);
            HeightMap* heigtMap = staticMesh->GetHeightMap();
            if (!(heigtMap->isValid() && staticMeshManager.IsValid(meshInnerHandle))) {
                MONA_LOG_ERROR("EnvironmentData: Saved terrain was not valid, so it was removed.");
                m_terrains.erase(m_terrains.begin() + i);
                i--;
            }
        }
    }

    float EnvironmentData::getTerrainHeight(glm::vec2 xyPoint, ComponentManager<TransformComponent>& transformManager,
        ComponentManager<StaticMeshComponent>& staticMeshManager) {
        float maxHeight = std::numeric_limits<float>::min();
        for (int i = 0; i < m_terrains.size(); i++) {
            if (!staticMeshManager.IsValid(m_terrains[i].m_meshHandle)) {
                m_terrains.erase(m_terrains.begin() + i);
                i--;
                MONA_LOG_WARNING("EnvironmentData: Saved terrain was not valid, so it was removed.");
                continue;
            }
            const StaticMeshComponent* staticMesh = staticMeshManager.GetComponentPointer(m_terrains[i].m_meshHandle);
            HeightMap* heigtMap = staticMesh->GetHeightMap();
            TransformComponent* staticMeshTransform = transformManager.GetComponentPointer(m_terrains[i].m_transformHandle);

            // trasladar punto a espacio local de la malla
            glm::vec4 basePoint(xyPoint, 0, 1);
            glm::vec4 meshPoint = glm::inverse(staticMeshTransform->GetModelMatrix())*basePoint;

            float localHeight = heigtMap->getHeight(meshPoint[0], meshPoint[1]);
            glm::vec4 localResult = { meshPoint[0], meshPoint[1], localHeight, 1 };
            // volver a espacio global
            glm::vec4 globalResult = staticMeshTransform->GetModelMatrix() * localResult;
            if (globalResult[2] > maxHeight) { maxHeight = globalResult[2]; }
        }
        return maxHeight;
    }

    void EnvironmentData::addTerrain(const GameObject& staticMeshObject) {
        Terrain terrain(staticMeshObject);
        m_terrains.push_back(terrain);  
    }

    int EnvironmentData::removeTerrain(const GameObject& staticMeshObject) {
        MONA_ASSERT(staticMeshObject.HasComponent<StaticMeshComponent>(), "EnvironmentData: Terrain object must have a satitc mesh component");
        InnerComponentHandle meshInnerHandle = staticMeshObject.GetInnerComponentHandle<StaticMeshComponent>();
        for (int i = 0; i < m_terrains.size(); i++) {
            if (m_terrains[i].m_meshHandle.m_generation == meshInnerHandle.m_generation && m_terrains[i].m_meshHandle.m_index==meshInnerHandle.m_index) {
                m_terrains.erase(m_terrains.begin() + i);
                return meshInnerHandle.m_index;
            }
        }
        return -1;
    }

}