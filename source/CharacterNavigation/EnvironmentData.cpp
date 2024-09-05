#include "EnvironmentData.hpp"
#include "../World/ComponentManager.hpp"
#include <limits>
#include <algorithm>
#include "../Core/Log.hpp"
#include <glm/glm.hpp>
namespace Mona{        

    Terrain::Terrain(const GameObjectHandle<GameObject>& staticMeshObject) {
        MONA_ASSERT(staticMeshObject->HasComponent<StaticMeshComponent>(), "EnvironmentData: Terrain object must have a satitc mesh component");
        m_transformHandle = staticMeshObject->GetInnerComponentHandle<TransformComponent>();
        m_meshHandle = staticMeshObject->GetInnerComponentHandle<StaticMeshComponent>();
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

    bool EnvironmentData::withinGlobalBoundaries(glm::vec2 xyPoint, HeightMap* heightMap, glm::mat4 globalTerrainTransform) {
        glm::vec2 globalMin = globalTerrainTransform * glm::vec4(heightMap->getMinXY(), 0, 1);
        glm::vec2 globalMax = globalTerrainTransform * glm::vec4(heightMap->getMaxXY(), 0, 1);
        return globalMin[0] <= xyPoint[0] && xyPoint[0] <= globalMax[0] &&
            globalMin[1] <= xyPoint[1] && xyPoint[1] <= globalMax[1];
    }

    float EnvironmentData::getTerrainHeight(glm::vec2 xyPoint, ComponentManager<TransformComponent>& transformManager,
        ComponentManager<StaticMeshComponent>& staticMeshManager) {
        float maxHeight = std::numeric_limits<float>::lowest();
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
            MONA_ASSERT(staticMeshTransform->GetLocalRotation() == glm::identity<glm::fquat>(), "EnvironmentData: Terrains cannot be rotated.");
            glm::mat4 glblTransform = staticMeshTransform->GetModelMatrix();

            if (withinGlobalBoundaries(xyPoint, heigtMap, glblTransform)) {
                // transformar punto a espacio local del terreno

                glm::vec3 localPoint = glm::inverse(glblTransform) * glm::vec4(xyPoint, 0, 1);
                float localHeight = heigtMap->getHeight(localPoint[0], localPoint[1]);
                localPoint[2] = localHeight;
                glm::vec4 glblPoint = glblTransform * glm::vec4(localPoint, 1);
                float result = glblPoint[2];
                if (result > maxHeight) { maxHeight = result; }
            }           
            
        }
        return maxHeight;
    }

    void EnvironmentData::addTerrain(const GameObjectHandle<GameObject>& staticMeshObject) {
        Terrain terrain(staticMeshObject);
        m_terrains.push_back(terrain);  
    }

    int EnvironmentData::removeTerrain(const GameObjectHandle<GameObject>& staticMeshObject) {
        MONA_ASSERT(staticMeshObject->HasComponent<StaticMeshComponent>(), "EnvironmentData: Terrain object must have a satitc mesh component");
        InnerComponentHandle meshInnerHandle = staticMeshObject->GetInnerComponentHandle<StaticMeshComponent>();
        for (int i = 0; i < m_terrains.size(); i++) {
            if (m_terrains[i].m_meshHandle.m_generation == meshInnerHandle.m_generation && m_terrains[i].m_meshHandle.m_index==meshInnerHandle.m_index) {
                m_terrains.erase(m_terrains.begin() + i);
                return meshInnerHandle.m_index;
            }
        }
        return -1;
    }

}