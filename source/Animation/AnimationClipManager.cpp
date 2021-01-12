#include "AnimationClipManager.hpp"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "../Core/AssimpTransformations.hpp"
#include "../Core/Log.hpp"
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

		//Si no hay un AnimationClip con la dirección entregada entonces se procese a cargar una nueva instancia de AnimationClip.

		Assimp::Importer importer;
		unsigned int postProcessFlags = aiProcess_Triangulate;
		const aiScene* scene = importer.ReadFile(stringPath, postProcessFlags);
		if (!scene || scene->mNumAnimations == 0)
		{
			MONA_LOG_ERROR("AnimationClipManager Error: Failed to open file with path {0}", stringPath);
			return nullptr;
		}
		
		//Solo cargamos la primera animación
		aiAnimation* animation = scene->mAnimations[0];
		// Dado que los tiempos de muestreo de assimp estan generalmente en ticks se debe dividir casi todos 
		// los tiempos de sus objetos por tickspersecond. Por otro lado, assimp sigue la convencion de que si mTicksPerSecond 
		// es 0.0 entonces los timeStamps de las muestras de las animaciones estan en segundos y no ticks.
		float ticksPerSecond = animation->mTicksPerSecond != 0.0f ? animation->mTicksPerSecond : 1.0f;
		float duration = animation->mDuration / animation->mTicksPerSecond;
		std::vector<AnimationClip::AnimationTrack> animationTracks;
		std::vector<std::string> trackNames;
		animationTracks.resize(animation->mNumChannels);
		trackNames.reserve(animation->mNumChannels);
		for (uint32_t i = 0; i < animation->mNumChannels; i++) {
			aiNodeAnim* track = animation->mChannels[i];
			trackNames.push_back(track->mNodeName.C_Str());
			AnimationClip::AnimationTrack& animationTrack = animationTracks[i];
			animationTrack.positions.reserve(track->mNumPositionKeys);
			animationTrack.positionTimeStamps.reserve(track->mNumPositionKeys);
			animationTrack.rotations.reserve(track->mNumRotationKeys);
			animationTrack.rotationTimeStamps.reserve(track->mNumRotationKeys);
			animationTrack.scales.reserve(track->mNumScalingKeys);
			animationTrack.scaleTimeStamps.reserve(track->mNumScalingKeys);
			for (uint32_t j = 0; j < track->mNumPositionKeys; j++) {
				animationTrack.positionTimeStamps.push_back(track->mPositionKeys[j].mTime / ticksPerSecond);
				animationTrack.positions.push_back(AssimpToGlmVec3(track->mPositionKeys[j].mValue));
			}
			for (uint32_t j = 0; j < track->mNumRotationKeys; j++) {
				animationTrack.rotationTimeStamps.push_back(track->mPositionKeys[j].mTime / ticksPerSecond);
				animationTrack.rotations.push_back(AssimpToGlmQuat(track->mRotationKeys[j].mValue));

			}
			for (uint32_t j = 0; j < track->mNumScalingKeys; j++) {
				animationTrack.scaleTimeStamps.push_back(track->mPositionKeys[j].mTime / ticksPerSecond);
				animationTrack.scales.push_back(AssimpToGlmVec3(track->mScalingKeys[j].mValue));

			}
		}
		AnimationClip* animationPtr = new AnimationClip(std::move(animationTracks),
			std::move(trackNames),
			skeleton,
			duration,
			ticksPerSecond,
			removeRootMotion);
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