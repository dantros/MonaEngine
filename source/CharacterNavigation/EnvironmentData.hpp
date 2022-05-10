#pragma once
#ifndef ENVIRONMENTDATA_HPP
#define ENVIRONMENTDATA_HPP

#include "MeshData.hpp"

namespace Mona {

	class EnvironmentData {
		private:
			std::vector<MeshData> m_terrains;
			std::vector<MeshData> m_obstacles;
			float _getTerrainHeight(float x, float y, MeshData terrain);
		public:
			float getTerrainHeight(float x, float y);
			void addTerrain(MeshData& terrain);
			int removeTerrain(MeshData& terrain);

	};

}

#endif