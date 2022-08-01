#pragma once
#ifndef MESH_HPP
#define MESH_HPP
#include <cstdint>
#include <string>
#include <glm/glm.hpp>
#include "../CharacterNavigation/HeightMap.hpp"

namespace Mona {
	class Mesh {
		friend class MeshManager;

	public:
		enum class PrimitiveType {
			Plane,
			Cube,
			Sphere,
			PrimitiveCount
		};
		~Mesh();
		uint32_t GetVertexArrayID() const noexcept { return m_vertexArrayID; }
		uint32_t GetIndexBufferCount() const noexcept { return m_indexBufferCount; }
		HeightMap* GetHeightMap() {
			if (m_heightMap.isValid() ) { return &m_heightMap; }
			else { return nullptr; }
		}
	private:
		Mesh(const std::string& filePath, bool flipUVs = false);
		Mesh(PrimitiveType type);
		Mesh(const glm::vec2& minXY, const glm::vec2& maxXY, int numInnerVerticesWidth, int numInnerVerticesHeight,
			float (*heightFunc)(float, float));

		void ClearData() noexcept;
		void CreateSphere() noexcept;
		void CreateCube() noexcept;
		void CreatePlane() noexcept;

		uint32_t m_vertexArrayID;
		uint32_t m_vertexBufferID;
		uint32_t m_indexBufferID;
		uint32_t m_indexBufferCount;
		HeightMap m_heightMap;
	};
}
#endif