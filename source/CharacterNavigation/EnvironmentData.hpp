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
			EnvironmentData(ComponentManager<TransformComponent>* transformManager, ComponentManager<StaticMeshComponent>* staticMeshManager);
			std::vector<Terrain> m_terrains;
			ComponentManager<TransformComponent>* m_transformManager = nullptr;
			ComponentManager<StaticMeshComponent>* m_staticMeshManager = nullptr;
		public:
			float getTerrainHeight(float x, float y);
			void addTerrain(const Terrain& terrain);
			int removeTerrain(const Terrain& terrain);
	};

}

#endif