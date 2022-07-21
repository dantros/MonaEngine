#pragma once
#ifndef ENVIRONMENTDATA_HPP
#define ENVIRONMENTDATA_HPP

#include "../World/GameObject.hpp"
#include "../World/TransformComponent.hpp"
#include "../Rendering/StaticMeshComponent.hpp"
#include "../World/GameObjectTypes.hpp"

namespace Mona {
	class Terrain {
		friend class EnvironmentData;
	private:
		Terrain(const GameObject& staticMeshObject);
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
			void addTerrain(const GameObject& staticMeshObject);
			int removeTerrain(const GameObject& staticMeshObject);
			void validateTerrains(ComponentManager<StaticMeshComponent>& staticMeshManager);
	};

}

#endif