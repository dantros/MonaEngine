#pragma once
#ifndef ENVIRONMENTDATA_HPP
#define ENVIRONMENTDATA_HPP

#include "HeightMap.hpp"
#include "../World/ComponentManager.hpp"
#include "../World/ComponentTypes.hpp"
#include "../World/ComponentHandle.hpp"
#include "../World/TransformComponent.hpp"
#include "../Rendering/StaticMeshComponent.hpp"

namespace Mona {
	class Terrain {
		friend class EnvironmentData;
	public:
		Terrain(const GameObject& staticMeshObject);
	private:
		InnerComponentHandle m_transformHandle;
		InnerComponentHandle m_meshHandle;
	};

	class EnvironmentData {
		private:
			friend class IKNavigationLifetimePolicy;
			EnvironmentData() = default;
			std::vector<Terrain> m_terrains;
		public:
			float getTerrainHeight(float x, float y, ComponentManager<TransformComponent>* transformManager, 
				ComponentManager<StaticMeshComponent>* staticMeshManager);
			void addTerrain(const Terrain& terrain, ComponentManager<StaticMeshComponent>* staticMeshManager);
			int removeTerrain(const Terrain& terrain, ComponentManager<StaticMeshComponent>* staticMeshManager);
	};

}

#endif