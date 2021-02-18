#pragma once
#ifndef MESHMANAGER_HPP
#define MESHMANAGER_HPP
#include <memory>
#include <filesystem>
#include <unordered_map>
#include "Mesh.hpp"
namespace Mona {

	class SkinnedMesh;
	class Skeleton;
	class MeshManager {
	public:
		friend class World;
		using MeshMap = std::unordered_map<std::string, std::shared_ptr<Mesh>>;
		using SkinnedMeshMap = std::unordered_map<std::string, std::shared_ptr<SkinnedMesh>>;
		MeshManager(MeshManager const&) = delete;
		MeshManager& operator=(MeshManager const&) = delete;
		std::shared_ptr<Mesh> LoadMesh(Mesh::PrimitiveType type) noexcept;
		std::shared_ptr<Mesh> LoadMesh(const std::filesystem::path& filePath, bool flipUVs = false) noexcept;
		std::shared_ptr<SkinnedMesh> LoadSkinnedMesh(std::shared_ptr<Skeleton> skeleton,
			const std::filesystem::path& filePath,
			bool flipUVs = false) noexcept;
		void CleanUnusedMeshes() noexcept;
		static MeshManager& GetInstance() noexcept{
			static MeshManager instance;
			return instance;
		}
	private:
		MeshManager() = default;
		void ShutDown() noexcept;
		MeshMap m_meshMap;
		SkinnedMeshMap m_skinnedMeshMap;

	};
}
#endif