#pragma once
#ifndef ENVIRONMENTDATA_HPP
#define ENVIRONMENTDATA_HPP

#include "../World/GameObjectHandle.hpp"
#include "../World/TransformComponent.hpp"
#include "../Rendering/StaticMeshComponent.hpp"
#include "../World/GameObjectTypes.hpp"

namespace Mona {
	class Terrain {
		friend class EnvironmentData;
	private:
		Terrain(const GameObjectHandle<GameObject>& staticMeshObject);
		InnerComponentHandle m_transformHandle;
		InnerComponentHandle m_meshHandle;
	};

	class EnvironmentData {
		private:
			std::vector<Terrain> m_terrains;
		public:
			EnvironmentData() = default;
			float getTerrainHeight(glm::vec2 xyPoint, ComponentManager<TransformComponent>& transformManager,
				ComponentManager<StaticMeshComponent>& staticMeshManager);
			void addTerrain(const GameObjectHandle<GameObject>& staticMeshObject);
			int removeTerrain(const GameObjectHandle<GameObject>& staticMeshObject);
			void validateTerrains(ComponentManager<StaticMeshComponent>& staticMeshManager);
	};

}

#endif