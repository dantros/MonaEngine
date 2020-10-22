#include "AudioClipManager.hpp"
#include "../Core/Log.hpp"
namespace Mona {
	std::shared_ptr<AudioClip> AudioClipManager::LoadAudioClip(const std::filesystem::path& filePath) noexcept {
		const std::string stringPath = filePath.string();
		auto it = m_audioClipMap.find(stringPath);
		if (it != m_audioClipMap.end())
			return it->second;
		AudioClip* audioClipPtr = new AudioClip(stringPath);
		std::shared_ptr<AudioClip> audioClipSharedPtr = std::shared_ptr<AudioClip>(audioClipPtr);
		m_audioClipMap.insert({ stringPath, audioClipSharedPtr});
		return audioClipSharedPtr;

	}

	void AudioClipManager::CleanUnusedAudioClips() noexcept {
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