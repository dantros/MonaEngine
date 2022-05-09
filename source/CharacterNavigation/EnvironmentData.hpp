#pragma once
#ifndef ENVIRONMENTDATA_HPP
#define ENVIRONMENTDATA_HPP

#include <vector>
#include "../World/ComponentHandle.hpp"
#include "../World/World.hpp"

namespace Mona {

	class EnvironmentData {

		private:
			std::vector<StaticMeshHandle> m_terrainHandles;
			std::vector<StaticMeshHandle> m_obstacleHandles;
			World m_world;
			float terrainHeight(StaticMeshHandle terrainHandle);
		public:
			void addTerrain(StaticMeshHandle& terrainHandle);
			int removeTerrain(StaticMeshHandle& terrainHandle);


	};




}







#endif