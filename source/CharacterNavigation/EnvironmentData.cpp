#include "EnvironmentData.hpp"
#include "../World/ComponentManager.hpp"
#include <limits>
#include <algorithm>
#include "../Core/Log.hpp"
#include "../Utilities/FuncUtils.hpp"
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

            // TODO
            // encontrar una linea perpendicular al plano xy de la malla y buscar su
            // interseccion con el plano xy global
            // encontrar el up vector adecuado para usar en el espacio local de la malla

            float result = heigtMap->getHeight(xyPoint[0], xyPoint[1]);
            if (result > maxHeight) { maxHeight = result; }
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