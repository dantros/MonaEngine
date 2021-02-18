#include "MeshManager.hpp"
#include "../Animation/SkinnedMesh.hpp"
namespace Mona {
	
	std::string PrimitiveEnumToString(Mesh::PrimitiveType type) {
		switch (type)
		{
		case Mona::Mesh::PrimitiveType::Plane:
			return "Plane";
			break;
		case Mona::Mesh::PrimitiveType::Cube:
			return "Cube";
			break;
		case Mona::Mesh::PrimitiveType::Sphere:
			return "Sphere";
			break;
		default:
			return "Sphere";
			break;
		}
	}

	std::shared_ptr<Mesh> MeshManager::LoadMesh(Mesh::PrimitiveType type) noexcept
	{
		const std::string primName = PrimitiveEnumToString(type);
		auto it = m_meshMap.find(primName);
		if (it != m_meshMap.end())
		{
			return it->second;
		}
		Mesh* meshPtr = new Mesh(type);
		std::shared_ptr<Mesh> sharedPtr = std::shared_ptr<Mesh>(meshPtr);
		//Antes de retornar la malla recien cargada, insertamos esta al mapa para que cargas futuras sean mucho mas rapidas.
		m_meshMap.insert({ primName, sharedPtr });
		return sharedPtr;
	}

	std::shared_ptr<Mesh> MeshManager::LoadMesh(const std::filesystem::path& filePath, bool flipUVs) noexcept {
		const std::string& stringPath = filePath.string();
		//En caso de que ya exista una entrada en el mapa de mallas con el mismo path, entonces se retorna inmediatamente
		//dicha malla.
		auto it = m_meshMap.find(stringPath);
		if (it != m_meshMap.end()) {
			return it->second;
		}
		Mesh* meshPtr = new Mesh(stringPath, flipUVs);
		std::shared_ptr<Mesh> sharedPtr = std::shared_ptr<Mesh>(meshPtr);
		//Antes de retornar la malla recien cargada, insertamos esta al mapa para que cargas futuras sean mucho mas rapidas.
		m_meshMap.insert({ stringPath, sharedPtr });
		return sharedPtr;

	}

	void MeshManager::CleanUnusedMeshes() noexcept {
		/*
		* Elimina todos los punteros del mapa de mallas cuyo conteo de referencias es igual a uno,
		* es decir, que el puntero del mapa es el unico que apunta a esa memoria.
		*/
		for (auto i = m_meshMap.begin(), last = m_meshMap.end(); i != last;) {
			if (i->second.use_count() == 1) {
				i = m_meshMap.erase(i);
			}
			else {
				++i;
			}

		}


		/*
		* Elimina todos los punteros del mapa de mallas para animación cuyo conteo de referencias es igual a uno,
		* es decir, que el puntero del mapa es el unico que apunta a esa memoria.
		*/
		for (auto i = m_skinnedMeshMap.begin(), last = m_skinnedMeshMap.end(); i != last;) {
			if (i->second.use_count() == 1) {
				i = m_skinnedMeshMap.erase(i);
			}
			else {
				++i;
			}

		}
	}
	void MeshManager::ShutDown() noexcept {
		for (auto& entry : m_meshMap) {
			entry.second->ClearData();
		}

		for (auto& entry : m_skinnedMeshMap) {
			entry.second->ClearData();
		}

		m_meshMap.clear();
	}

	std::shared_ptr<SkinnedMesh> MeshManager::LoadSkinnedMesh(std::shared_ptr<Skeleton> skeleton,
		const std::filesystem::path& filePath,
		bool flipUVs) noexcept 
	{
		const std::string& stringPath = filePath.string();
		//En caso de que ya exista una entrada en el mapa de mallas para animación con el mismo path, 
		// entonces se retorna inmediatamente dicha malla.
		auto it = m_skinnedMeshMap.find(stringPath);
		if (it != m_skinnedMeshMap.end()) {
			return it->second;
		}
		SkinnedMesh* meshPtr = new SkinnedMesh(skeleton, stringPath, flipUVs);
		std::shared_ptr<SkinnedMesh> sharedPtr = std::shared_ptr<SkinnedMesh>(meshPtr);
		//Antes de retornar la malla recien cargada, insertamos esta al mapa para que cargas futuras sean mucho mas rapidas.
		m_skinnedMeshMap.insert({ stringPath, sharedPtr });
		return sharedPtr;

	}
}