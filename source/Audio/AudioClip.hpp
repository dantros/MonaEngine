#pragma once
#ifndef AUDIOCLIP_HPP
#define AUDIOCLIP_HPP
#include <string>
#include <AL/al.h>
#include <AL/alc.h>
namespace Mona {

	/*
		Clase que representa una pista de audio (Efectos de sonido y/o Musica), de momento solo soporta formato .wav.
	*/
	class AudioClip {
	public:
		/*
		* La clase AudioClipManager es declarada como amiga, ya que el constructor es privado,
		* y es mediante la interfaz de este manager que es posible crear instancias de AudioClip.
		*/
		friend class AudioClipManager;
		
		/*
		* Dado que esta clase contiene recursos que son destruidos durante el llamado al destructor,
		* para evitar liberación de memoria ya destruida es necesario eliminar tanto el copy constructor
		* como el ooperador de copia.
		*/
		AudioClip(const AudioClip&) = delete;
		AudioClip& operator=(const AudioClip&) = delete;

		/*
		* Retorna el número identificador del buffer de OpenAL.
		*/
		ALuint GetBufferID() const { return m_alBufferID; }

		/*
		* Retorna la duración en segundos de este AudioClip
		*/
		float GetTotalTime() const { return m_totalTime; }

		/*
		* Retorna el número de canales de este AudioClip (Mono = 2, Estereo = 2).
		*/
		uint8_t GetChannels() const { return m_channels; }

		/*
		* Retorna la frecuencias de muestreo de este AudioClip
		*/
		uint32_t GetSampleRate() const { return m_sampleRate; }
		~AudioClip();
	private:

		/*
		* Contruye una instancia de AudioClip a partir de un string que contiene la dirección del archivo
		* con los datos de audio (EJ: "C:/Home/Desktop/Music.wav") . De momento el único formato soporta es wav.
		*/
		AudioClip(const std::string& audioFilePath);

		/*
		* Metodo que libera los recursos de OpenAL asociados a esta instancia. Esta función es llamada al momento
		* que el motor esta preparandose para ser cerrado.
		*/
		void DeleteOpenALBuffer();
		
		uint32_t m_sampleRate;
		float m_totalTime;
		ALuint m_alBufferID;
		uint8_t m_channels;
	};
}
#endif