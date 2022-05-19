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
	public:
		Terrain(InnerComponentHandle transformHandle, InnerComponentHandle meshHandle);
	private:
		InnerComponentHandle m_transformHandle;
		InnerComponentHandle m_meshHandle;
	};

	class EnvironmentData {
		private:
			friend class IKNavigationLifetimePolicy;
			EnvironmentData(ComponentManager<TransformComponent>* transformManager, ComponentManager<StaticMeshComponent>* staticMeshManager, 
				ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr);
			std::vector<InnerComponentHandle> m_terrains;
			ComponentManager<TransformComponent>* m_transformManager = nullptr;
			ComponentManager<StaticMeshComponent>* m_staticMeshManager = nullptr;
			ComponentManager<RigidBodyComponent>* m_rigidBodyManagerPtr = nullptr;
		public:
			float getTerrainHeight(float x, float y);
			void addTerrain(const StaticMeshHandle& terrain);
			int removeTerrain(const StaticMeshHandle& terrain);
	};

}

#endif