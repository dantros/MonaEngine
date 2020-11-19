#pragma once
#ifndef AUDIOCLIPMANAGER_HPP
#define AUDIOCLIPMANAGER_HPP
#include <memory>
#include <unordered_map>
#include <filesystem>
#include <string>
#include "AudioClip.hpp"
namespace Mona {
	/*
	* Clase responsable de la creación y administración de instancias de AudioClips
	*/
	class AudioClipManager {
	public:
		friend class World;
		using AudioClipMap = std::unordered_map<std::string, std::shared_ptr<AudioClip>>;
		AudioClipManager(AudioClipManager const&) = delete;
		AudioClipManager& operator=(AudioClipManager const&) = delete;
		/*
		* Crea o obtiene una instancia de AudioClip asociada al archivo ubicado en filePath.
		* Si ya se cargo un AudioClip con la misma ubicación el proceso de construccion de la instancia
		* de AudioClip sera omitida y se entregara un puntero a una instancia previamente creada.
		*/
		std::shared_ptr<AudioClip> LoadAudioClip(const std::filesystem::path& filePath) noexcept;
		/*
		* Limpia o elimina las instancias de AudioCLips que solo estan siendo referenciadas por esta clase
		*/
		void CleanUnusedAudioClips() noexcept;

		static AudioClipManager& GetInstance() noexcept {
			static AudioClipManager instance;
			return instance;
		}

	private:
		AudioClipManager() = default;
		/*
		* Este metodo es llamado al momento que el motor se esta preparando para ser cerrado. Y se encarga de
		* liberar todos los recursos de OpenAL asoaciados a cada una de las instancias de AudioClip cargadas.
		*/
		void ShutDown() noexcept;
		AudioClipMap m_audioClipMap;
	};
}
#endif