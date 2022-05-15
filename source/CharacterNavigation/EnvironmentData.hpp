#pragma once
#ifndef ENVIRONMENTDATA_HPP
#define ENVIRONMENTDATA_HPP

#include "HeightMap.hpp"

namespace Mona {

	class EnvironmentData {
		private:
			std::vector<HeightMap*> m_terrains;
		public:
			float getTerrainHeight(float x, float y);
			void addTerrain(HeightMap* terrain);
			int removeTerrain(HeightMap* terrain);
	};

}

#endif