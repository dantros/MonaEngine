#pragma once
#ifndef MESH_HPP
#define MESH_HPP
#include <cstdint>
#include <glad/glad.h>
#include "../Core/Log.hpp"
namespace Mona {
	class Mesh {
		friend class MeshManager;
	public:
		Mesh(uint32_t vaoID, uint32_t vboID, uint32_t iboID, uint32_t count) : 
			m_vaoID(vaoID),
			m_vboID(vboID),
			m_iboID(iboID),
			m_count(count) {}
		~Mesh() {
			MONA_ASSERT(m_vaoID, "Mesh Error: Trying to delete already deleted mesh");
			MONA_ASSERT(m_vboID, "Mesh Error: Trying to delete already deleted mesh");
			MONA_ASSERT(m_iboID, "Mesh Error: Trying to delete already deleted mesh");
			glDeleteBuffers(1, &m_vboID);
			glDeleteBuffers(1, &m_iboID);
			glDeleteVertexArrays(1, &m_vaoID);
		}
		uint32_t GetVAOID() const noexcept { return m_vaoID; }
		uint32_t GetCount() const noexcept { return m_count; }
	private:
		void ClearData() noexcept {
			MONA_ASSERT(m_vaoID, "Mesh Error: Trying to delete already deleted mesh");
			MONA_ASSERT(m_vboID, "Mesh Error: Trying to delete already deleted mesh");
			MONA_ASSERT(m_iboID, "Mesh Error: Trying to delete already deleted mesh");
			glDeleteBuffers(1, &m_vboID);
			glDeleteBuffers(1, &m_iboID);
			glDeleteVertexArrays(1, &m_vaoID);

		}

		uint32_t m_vaoID;
		uint32_t m_vboID;
		uint32_t m_iboID;
		uint32_t m_count;
	};
}
#endif