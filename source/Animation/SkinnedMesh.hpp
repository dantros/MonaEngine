#pragma once
#ifndef SKINNEDMESH_HPP
#define SKINNEDMESH_HPP
#include <cstdint>
#include <memory>
#include <string>
namespace Mona {
	class Skeleton;
	class SkinnedMesh {
		friend class MeshManager;
	public:
		~SkinnedMesh();
		uint32_t GetVertexArrayID() const noexcept { return m_vertexArrayID; }
		uint32_t GetIndexBufferCount() const noexcept { return m_indexBufferCount; }
		std::shared_ptr<Skeleton> GetSkeleton() const noexcept{ return m_skeletonPtr; }
	private:
		SkinnedMesh(std::shared_ptr<Skeleton> skeleton,
			const std::string& filePath,
			bool flipUvs = false);
		void ClearData() noexcept;
		std::shared_ptr<Skeleton> m_skeletonPtr;
		uint32_t m_vertexArrayID;
		uint32_t m_vertexBufferID;
		uint32_t m_indexBufferID;
		uint32_t m_indexBufferCount;
	};
}
#endif