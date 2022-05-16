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
	class EnvironmentData {
		private:
			std::vector<InnerComponentHandle> m_terrains;
			ComponentManager<TransformComponent>* m_transformManager;
			ComponentManager<StaticMeshComponent>* m_staticMeshManager;
		public:
			float getTerrainHeight(float x, float y);
			void addTerrain(const StaticMeshHandle& terrain);
			int removeTerrain(const StaticMeshHandle& terrain);
	};

}

#endif