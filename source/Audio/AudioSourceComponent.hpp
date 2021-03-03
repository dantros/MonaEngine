#pragma once
#ifndef AUDIOSOURCECOMPONENT_HPP
#define AUDIOSOURCECOMPONENT_HPP
#include <memory>
#include "../World/GameObjectTypes.hpp"
#include "AudioClip.hpp"
#include "AudioSource.hpp"
#include "../World/ComponentTypes.hpp"


namespace Mona {
	enum class AudioSourceState {
		Stopped,
		Playing,
		Paused
	};

	class AudioSourceComponentLifetimePolicy;
	class AudioSourceComponent : public AudioSource {
	public:
		//using managerType = ComponentManager<AudioSourceComponent, AudioSourceComponentLifetimePolicy>;
		using LifetimePolicyType = AudioSourceComponentLifetimePolicy;
		using dependencies = DependencyList<TransformComponent>;
		static constexpr std::string_view componentName = "AudioSourceComponent";
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::AudioSourceComponent);

		friend class AudioSystem;
		friend class AudioSourceComponentLifetimePolicy;
		AudioSourceComponent(std::shared_ptr<AudioClip> audioClip = nullptr,
			float volume = 1.0f,
			float pitch = 1.0f,
			bool isLooping = false,
			float radius = 1000.0f,
			AudioSourcePriority priority = AudioSourcePriority::SoundPriorityMedium,
			SourceType sourceType = SourceType::Source2D);

		/*
		* Retorna la instancia de AudioClip asignado a esta fuente de audio.
		*/
		std::shared_ptr<AudioClip> GetAudioClip() const noexcept { return m_audioClip; }

		/*
		* Ajusta la instancia de AudioClip de esta fuente de audio. Tambien detiene la reproducción del
		* clip anterior.
		*/
		void SetAudioClip(std::shared_ptr<AudioClip> audioClip) noexcept;

		/*
		* Comienza a reproducir el audioclip actual como si fuera emitido por esta fuente de audio. En caso
		* de que la fuente ya se encontraba reproduciendo este clip la reproducción vuelve a empezar del comienzo.
		*/
		void Play() noexcept;

		/*
		* Detiene completamente la reproduccin de audio de esta fuente. La proxima vez que se llame a Play, la reproducción
		* comenzara desde el principio del clip de audio.
		*/
		void Stop() noexcept;

		/*
		* Detiene la reproducción de audio de esta fuente. La proxima vez que se llame a Play la reproducción sera retomada
		* en el mismo lugar donde se pauso.
		*/
		void Pause() noexcept;


		AudioSourceState GetAudioSourceState() const noexcept { return m_sourceState; }
		AudioSourcePriority GetSourcePriority() const noexcept { return m_priority; }
		void SetSourcePriority(AudioSourcePriority priority) noexcept { m_priority = priority; }
		SourceType GetSourceType() const noexcept { return m_sourceType; }
		void SetSourceType(SourceType type) noexcept;


		/*
		* Retorna el valor del volumen de esta fuente.
		*/
		float GetVolume() const noexcept { return m_volume; }
		/*
		* Ajusta el valor del volumen de esta fuente. Internamente el valor entregado es restringido al rango 0.0f-1.0f.
		*/
		void SetVolume(float volume) noexcept;
		
		/*
		* Retorna el radio de alcance de esta fuente de audio. Para fuentes 3D a mayor radio de alcance mas lento es el decaimiento del volumen en función
		* de la distancia al receptor, fuentes de audio a una distancia mayor que este radio no reproducen sonido.
		*/
		float GetRadius() const noexcept { return m_radius; }

		/*
		* Ajusta el radio de alcance de esta fuente de audio. Internamente el valor es restringido a valores positivos.
		*/
		void SetRadius(float radius) noexcept;
		
		/*
		* Retorna el tono de esta fuente de sonido
		*/
		float GetPitch() const noexcept { return m_pitch; }
		/*
		* Ajusta el tono de esta fuente de sonido, internamente el valor entregado es restringido a valores mayores que cero. 
		*/
		void SetPitch(float pitch) noexcept;

		/*
		* Retorna verdadero si el audio clip reproducido por esta fuente esta en repetición.
		*/
		bool IsLooping() const noexcept { return m_isLooping; }

		/*
		* Ajusta si el clip de audio siendo reproducido por esta fuente se repite una vez termina su 
		* reproducción.
		*/
		void SetIsLooping(bool looping) noexcept;

		/*
		* Retorna el tiempo en segundos que le quedan por reproducir al actual clip de audio 
		* asociado a este fuente de Audio.
		*/
		float GetTimeLeft() const noexcept { return m_timeLeft; }


	private:
		void SetTransformHandle(const InnerComponentHandle& handle) { m_transformHandle = handle; }
		InnerComponentHandle m_transformHandle;
		AudioSourceState m_sourceState;
		bool m_isLooping;
	};
}
#endif