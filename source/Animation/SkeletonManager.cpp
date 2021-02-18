#include "SkeletonManager.hpp"
#include "Skeleton.hpp"
namespace Mona {
	std::shared_ptr<Skeleton> SkeletonManager::LoadSkeleton(const std::filesystem::path& filePath) noexcept {
		const std::string& stringPath = filePath.string();
		//En caso de que ya exista una entrada en el mapa de esqueletos con el mismo path, 
		// entonces se retorna inmediatamente dicho esqueleto.
		auto it = m_skeletonMap.find(stringPath);
		if (it != m_skeletonMap.end()) {
			return it->second;
		}

		Skeleton* skeletonPtr = new Skeleton(stringPath);
		std::shared_ptr<Skeleton> skeletonSharedPtr = std::shared_ptr<Skeleton>(skeletonPtr);
		m_skeletonMap.insert({stringPath, skeletonSharedPtr});
		return skeletonSharedPtr;
	}

	void SkeletonManager::CleanUnusedSkeletons() noexcept {
		/*
		* Elimina todos los punteros del mapa cuyo conteo de referencias es igual a uno,
		* es decir, que el puntero del mapa es el unico que apunta a esa memoria.
		*/
		for (auto i = m_skeletonMap.begin(), last = m_skeletonMap.end(); i != last;) {
			if (i->second.use_count() == 1) {
				i = m_skeletonMap.erase(i);
			}
			else {
				++i;
			}

		}
	}

	void SkeletonManager::ShutDown() noexcept {
		//Al cerrar el motor se llama esta función donde se limpia el mapa de equeletos
		m_skeletonMap.clear();
	}
}