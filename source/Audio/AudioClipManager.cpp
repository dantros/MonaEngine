#include "AudioClipManager.hpp"
#include "../Core/Log.hpp"
namespace Mona {
	std::shared_ptr<AudioClip> AudioClipManager::LoadAudioClip(const std::filesystem::path& filePath) noexcept {
		const std::string stringPath = filePath.string();
		//Primero se chequea si ya hay una instancia en el mapa de AudioClip con la misma dirección recien entregada
		auto it = m_audioClipMap.find(stringPath);
		if (it != m_audioClipMap.end())
			return it->second;
		//Si no hay un AudioClip con la dirección entregada entonces se procese a cargar una nueva instancia de AudioClip.
		AudioClip* audioClipPtr = new AudioClip(stringPath);
		std::shared_ptr<AudioClip> audioClipSharedPtr = std::shared_ptr<AudioClip>(audioClipPtr);
		m_audioClipMap.insert({ stringPath, audioClipSharedPtr});
		return audioClipSharedPtr;

	}

	void AudioClipManager::CleanUnusedAudioClips() noexcept {
		//Se recorre el mapa de AudioClips revisando los punteros compartidos que tienen un conteo de referencias igual a uno,
		//es decir, que solo es este mapa quien los referencia.
		for(auto i = m_audioClipMap.begin(), last = m_audioClipMap.end(); i!= last;){
			if(i->second.use_count() == 1){
				i = m_audioClipMap.erase(i);
			}
			else{
				++i;
			}

		}
	}

	void AudioClipManager::ShutDown() noexcept {
		for (auto& entry : m_audioClipMap) {
			(entry.second)->DeleteOpenALBuffer();
		}
		m_audioClipMap.clear();
	}
}