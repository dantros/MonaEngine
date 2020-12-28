#pragma once
#ifndef SKINNEDMESH_HPP
#define SKINNEDMESH_HPP
#include <cstdint>
#include <memory>
#include <glad/glad.h>
#include "../Core/Log.hpp"
namespace Mona {
	class Skeleton;
	class SkinnedMesh {
		friend class MeshManager;
	public:
		~SkinnedMesh() {
			if (m_vaoID)
				ClearData();
		}
		uint32_t GetVAOID() const noexcept { return m_vaoID; }
		uint32_t GetCount() const noexcept { return m_count; }
		std::shared_ptr<Skeleton> GetSkeleton() const noexcept{ return m_skeletonPtr; }
	private:
		SkinnedMesh(std::shared_ptr<Skeleton> skeleton, uint32_t vaoID, uint32_t vboID, uint32_t iboID, uint32_t count) :
			m_skeletonPtr(skeleton),
			m_vaoID(vaoID),
			m_vboID(vboID),
			m_iboID(iboID),
			m_count(count) {}
		void ClearData() noexcept {
			MONA_ASSERT(m_vaoID, "SkinnedMesh Error: Trying to delete already deleted skinnedMesh");
			MONA_ASSERT(m_vboID, "SkinnedMesh Error: Trying to delete already deleted skinnedMesh");
			MONA_ASSERT(m_iboID, "SkinnedMesh Error: Trying to delete already deleted skinnedMesh");
			glDeleteBuffers(1, &m_vboID);
			glDeleteBuffers(1, &m_iboID);
			glDeleteVertexArrays(1, &m_vaoID);
			m_vaoID = 0;

		}
		std::shared_ptr<Skeleton> m_skeletonPtr;
		uint32_t m_vaoID;
		uint32_t m_vboID;
		uint32_t m_iboID;
		uint32_t m_count;
	};
}
#endif