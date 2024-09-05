#pragma once
#ifndef HEIGHTMAP_HPP
#define HEIGHTMAP_HPP

#include <vector>
#include <glm/glm.hpp>
#include <unordered_map>

namespace Mona {

	class HeightMap{
		friend class EnvironmentData;
		private:
			float m_minX;
			float m_minY;
			float m_maxX;
			float m_maxY;
			float (*m_heightFunc)(float, float) = nullptr;

		public:
			HeightMap() = default;
			HeightMap(const glm::vec2& bottomLeft, const glm::vec2& topRight, float (*heightFunc)(float, float));
			bool withinBoundaries(float x, float y);
			glm::vec2 getMinXY() { return glm::vec2( m_minX, m_minY ); }
			glm::vec2 getMaxXY() { return glm::vec2(m_maxX, m_maxY); }
			float getHeight(float x, float y);
			bool isValid() { return m_heightFunc != nullptr; }
	};

}


#endif