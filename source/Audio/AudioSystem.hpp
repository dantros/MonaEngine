#pragma once
#ifndef AUDIOSYSTEM_HPP
#define AUDIOSYSTEM_HPP
#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include <AL/al.h>
#include <AL/alc.h>
#include "../World/ComponentTypes.hpp"
#include "../World/TransformComponent.hpp"
#include "AudioClip.hpp"
#include "AudioSource.hpp"
#include "AudioSourceComponent.hpp"
namespace Mona {
	struct InnerComponentHandle;
	/*
	* Clase responsable de la logica del sistema de audio del motor.
	*/
	class AudioSystem {
	public:
		/*
		* Función llamada por el motor que inicializa el sistema de audio, el cual es capaz de reproducir channels audios simultaneamente.
		*/
		void StartUp() noexcept;
		
		/*
		* Libera todos los recuros mantenidos por el sistema de audio, esta función es llamada durante el proceso de cierre del motor.
		*/
		void ShutDown() noexcept;

		/*
		* Función llamada cada iteración del mainloop del motor
		*/
		void Update(const InnerComponentHandle &audioListenerTransformHandle,
			const glm::fquat &audioListenerOffsetRotation,
			float timeStep,
			const ComponentManager<TransformComponent>& transformDataManager,
			ComponentManager<AudioSourceComponent>& audioSourceDataManager) noexcept;

		/*
		* Retorna el volumen global o maestro del sistema de audio
		*/
		float GetMasterVolume() const noexcept;

		/*
		* Ajusta el volumen global o maestro del sistema de audio al valor entregado. Internamente el rango del valor es restringido a
		* valores entre 0.0f y 1.0f.
		*/
		void SetMasterVolume(float volume) noexcept;

		/*
		* Crea una FreeAudioSource (fuente libre) la cual sera responsable de intertar reproducir el audioclip entregado con la posición,
		* volumen, tono, radio y prioridad tambien entregados. Esta funcion suele ser llamada por medio de la interfaz de la clase World, cuando
		* el usuerio quiere reproducir un audio3D que no este unido como component a un GameObject.
		*/
		void PlayAudioClip3D(std::shared_ptr<AudioClip> audioClip,
			const glm::vec3& position,
			float volume,
			float pitch,
			float radius,
			AudioSourcePriority priority
			);
		/*
		* Crea una FreeAudioSource (fuente libre) la cual sera responsable de intertar reproducir el audioclip entregado con el volumen, 
		* tono y prioridad tambien entregados. Esta funcion suele ser llamada por medio de la interfaz de la clase World, cuando
		* el usuerio quiere reproducir un audio2D que no este unido como component a un GameObject.
		*/
		void PlayAudioClip2D(std::shared_ptr<AudioClip> audioClip,
			float volume,
			float pitch,
			AudioSourcePriority priority);

		/*
		* Libera la fuente de OpenAL que se encuentra en la posición index dentro del arreglo de estas mismas. Esta función es llamada
		* cada vez que una componente AudioSourceComponent es destruida.
		*/
		void RemoveOpenALSource(uint32_t index) noexcept;

		/*
		* Libera todas las fuentes de OpenAL
		*/
		void ClearSources() noexcept;
	private:

		void UpdateListener(const glm::vec3& position, const glm::vec3& frontVector, const glm::vec3& upVector);
		void RemoveCompletedFreeAudioSources();
		void UpdateFreeAudioSourcesTimers(float timeStep);
		void UpdateAudioSourceComponentsTimers(float timeStep, ComponentManager<AudioSourceComponent>& audioDataManager);
		std::vector<FreeAudioSource>::iterator PartitionAndRemoveOpenALSourceFromFreeAudioSources(const glm::vec3& listenerPosition);
		uint32_t PartitionAndRemoveOpenALSourceFromAudioSourceComponents(ComponentManager<AudioSourceComponent>& audioDataManager,
			const ComponentManager<TransformComponent>& transformDataManager,
			const glm::vec3& listenerPosition);
		void AssignOpenALSourceToFreeAudioSources(std::vector<FreeAudioSource>::iterator begin,
			std::vector<FreeAudioSource>::iterator end);
		void AssignOpenALSourceToAudioSourceComponents(ComponentManager<AudioSourceComponent>& audioDataManager,
			const ComponentManager<TransformComponent>& transformDataManager,
			uint32_t firstIndex,
			uint32_t lastIndex);
		void RemoveOpenALSourceFromFreeAudioSources(std::vector<FreeAudioSource>::iterator begin,
			std::vector<FreeAudioSource>::iterator end);
		void RemoveOpenALSourceFromAudioSourceComponents(ComponentManager<AudioSourceComponent>& audioDataManager,
			uint32_t firstIndex,
			uint32_t lastIndex);
		void SortFreeAudioSourcesByPriority(std::vector<FreeAudioSource>::iterator end,
			uint32_t (&outCount)[static_cast<unsigned int>(AudioSourcePriority::PriorityCount)]);
		void SortAudioSourceComponentsByPriority(ComponentManager<AudioSourceComponent>& audioDataManager,
			uint32_t lastIndex,
			uint32_t (&outCount)[static_cast<unsigned int>(AudioSourcePriority::PriorityCount)]);
		

	
		void FreeOpenALSource(uint32_t index);
		struct OpenALSourceArrayEntry {
			ALuint m_sourceID;
			uint32_t m_nextFreeIndex;
			OpenALSourceArrayEntry(ALuint source, uint32_t nextFreeIndex) :
				m_sourceID(source), m_nextFreeIndex(nextFreeIndex) {}
		};
		AudioSource::OpenALSource GetNextFreeSource();


		ALCcontext* m_audioContext;
		ALCdevice* m_audioDevice;
		std::vector<OpenALSourceArrayEntry> m_openALSources;
		uint32_t m_firstFreeOpenALSourceIndex;
		uint32_t m_channels;
		std::vector<FreeAudioSource> m_freeAudioSources;
		float m_masterVolume;
	};
}
#endif