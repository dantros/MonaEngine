#include "AnimationClipManager.hpp"
#include "AnimationClip.hpp"
#include "Skeleton.hpp"
namespace Mona {
	std::shared_ptr<AnimationClip> AnimationClipManager::LoadAnimationClip(const std::filesystem::path& filePath,
		std::shared_ptr<Skeleton> skeleton,
		bool removeRootMotion) noexcept
	{
		const std::string& stringPath = filePath.string();
		//En caso de que ya exista una entrada en el mapa de animaciones con el mismo path, 
		// entonces se retorna inmediatamente dicha animación.
		auto it = m_animationClipMap.find(stringPath);
		if (it != m_animationClipMap.end()) {
			return it->second;
		}

		AnimationClip* animationPtr = new AnimationClip(stringPath, skeleton, removeRootMotion);
		std::shared_ptr<AnimationClip> sharedPtr = std::shared_ptr<AnimationClip>(animationPtr);
		m_animationClipMap.insert({ stringPath, sharedPtr });
		return sharedPtr;
	}
	void AnimationClipManager::CleanUnusedAnimationClips() noexcept {
		/*
		* Elimina todos los punteros del mapa cuyo conteo de referencias es igual a uno,
		* es decir, que el puntero del mapa es el unico que apunta a esa memoria.
		*/
		for (auto i = m_animationClipMap.begin(), last = m_animationClipMap.end(); i != last;) {
			if (i->second.use_count() == 1) {
				i = m_animationClipMap.erase(i);
			}
			else {
				++i;
			}

		}
	}

	void AnimationClipManager::ShutDown() noexcept {
		//Al cerrar el motor se llama esta función donde se limpia el mapa de animaciones
		m_animationClipMap.clear();
	}

	
}