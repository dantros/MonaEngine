#pragma once
#ifndef AUDIOSOURCE_HPP
#define AUDIOSOURCE_HPP
#include <memory>
#include <optional>
#include <glm/glm.hpp>
#include "AudioClip.hpp"
#include <AL/al.h>
#include  <algorithm>
namespace Mona {
	class AudioClip;

	/*
	* Enumerador que representa la prioridad que tiene una fuente de sonido. En presencia de escazes
	* de recursos de OpenAL, es decir, cuando existen mas sonidos intentando ser repreducidos simultaneamente
	* que recursos para reproducirlos se le otorgaran primero estos primero a las fuentes de audio con mayor prioridad
	*/
	enum class AudioSourcePriority : uint8_t {
		SoundPriorityVeryHigh,
		SoundPriorityHigh,
		SoundPriorityMedium,
		SoundPriorityLow,
		SoundPriorityVeryLow,
		PriorityCount
	};

	/*
	* Enumerador que representa los tipos de fuentes de sonido. Las fuentes 2D se escuchan de igual manera independiente
	* de la posición/velocidad relativa de la fuente y el receptor, en cambio, las fuentes 3D si se ven afectadas por estas
	* variables.
	*/
	enum class SourceType : uint8_t{
		Source2D,
		Source3D
	};

	/*
	* El motor cuenta con 2 clases de Fuentes de sonido FreeSources y AudioSourceComponents, cuya principal diferencia es 
	* que FreeSources no estan relacionadas o unidas a ningun GameObject. Esta clase, AudioSource, es la clase base de ambas fuentes de
	* sonido.
	*/
	struct AudioSource {
		friend class  AudioSystem;
		struct OpenALSource {
			ALuint m_sourceID;
			uint32_t m_sourceIndex;
			OpenALSource(ALuint source = 0, uint32_t index = 0) : m_sourceID(source), m_sourceIndex(index) {}
		};
		AudioSource(std::shared_ptr<AudioClip> audioClip,
			float volume,
			float pitch,
			float radius,
			AudioSourcePriority priority,
			SourceType sourceType) :
			m_audioClip(audioClip),
			m_openALsource(std::nullopt),
			m_volume(volume),
			m_pitch(pitch),
			m_radius(radius),
			m_priority(priority),
			m_sourceType(sourceType),
			m_timeLeft(0.0f)
		{
			if(m_audioClip)
			{
				m_timeLeft = m_audioClip->GetTotalTime();
			}
		}
		protected:
		std::optional<OpenALSource> m_openALsource;
		std::shared_ptr<AudioClip> m_audioClip;
		float m_volume;
		float m_pitch;
		float m_radius;
		float m_timeLeft;
		AudioSourcePriority m_priority;
		SourceType m_sourceType;
		
	};

	/*
	* Clase que representa fuentes de Audio libres, es decir, que no estan unidas a un GameObject. En Caso de ser fuentes de audio 3D
	* al momento de construccion se le puede asigna una posicion constante.
	*/
	class FreeAudioSource : public AudioSource {
	public:
		friend class  AudioSystem;
		FreeAudioSource(std::shared_ptr<AudioClip> audioClip = nullptr,
			float volume = 1.0f,
			float pitch = 1.0f,
			float radius = 1000.0f,
			AudioSourcePriority priority = AudioSourcePriority::SoundPriorityMedium,
			SourceType sourceType = SourceType::Source2D,
			const glm::vec3 &position = glm::vec3(0.0f,0.0f,0.0f)) :
			AudioSource(audioClip, volume, pitch, radius, priority, sourceType),
			m_position(position)
		{}
	private:
		glm::vec3 m_position;
	};
	
}
#endif