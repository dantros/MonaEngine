#pragma once
#ifndef AUDIOSOURCECOMPONENTLIFETIMEPOLICY_HPP
#define AUDIOSOURCECOMPONENTLIFETIMEPOLICY_HPP
#include "AudioSystem.hpp"
#include "AudioSourceComponent.hpp"
namespace Mona {
	class GameObject;
	/*
	* Clase que representa las polizas de agregar y remover una instancia de AudioSourceComponent a un GameObject, es decir,
	* cada vez que se agrega una de esta componente el metodo OnAddComponent es llamado, mientras que cuando se remueve
	* OnRemoveComponent es llamado
	*/
	class AudioSourceComponentLifetimePolicy {
	public:
		AudioSourceComponentLifetimePolicy() = default;
		AudioSourceComponentLifetimePolicy(AudioSystem* audioSystem) : m_audioSystem(audioSystem) {}
		void OnAddComponent(GameObject* gameObjectPtr, AudioSourceComponent& audioSource, const InnerComponentHandle& handle) {
			//Configura el vamor del handle que representa la transformada para asi poder obtener la información espacial necesaria
			// para el sistema de audio a partir de la transformada del GameObject al que se le esta agregando esta componente
			audioSource.SetTransformHandle(gameObjectPtr->GetInnerComponentHandle<TransformComponent>());
		}
		void OnRemoveComponent(GameObject* gameObjectPtr,AudioSourceComponent& audioSource, const InnerComponentHandle& handle) {
			//En caso de que la componente siendo removida esta usando un recurso de OpenAL, este es liberado.
			if (audioSource.m_openALsource) {
				const auto& alSource = audioSource.m_openALsource.value();
				m_audioSystem->RemoveOpenALSource(alSource.m_sourceIndex);

			}
		}

	private:
		AudioSystem* m_audioSystem;
	};
}
#endif