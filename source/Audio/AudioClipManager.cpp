#include "AudioClipManager.hpp"
#include "../Core/Log.hpp"
namespace Mona {
	std::shared_ptr<AudioClip> AudioClipManager::LoadAudioClip(const std::filesystem::path& filePath) noexcept {
		const std::string stringPath = filePath.string();
		auto& it = m_audioClipMap.find(stringPath);
		if (it != m_audioClipMap.end())
			return it->second.lock();
		std::shared_ptr<AudioClip> audioClip = std::make_shared<AudioClip>(stringPath);
		m_audioClipMap.insert({ stringPath, std::weak_ptr(audioClip) });
		return audioClip;

	}

	void AudioClipManager::CleanUnusedAudioClips() noexcept {
		std::erase_if(m_audioClipMap, [](const auto& item) {
			auto const& [key, value] = item;
			return value.expired();
			});
	}

	void AudioClipManager::ShutDown() noexcept {
		for (auto& entry : m_audioClipMap) {
			auto audioClip = entry.second.lock();
			if(audioClip)
				audioClip->DeleteOpenALBuffer();
		}
		m_audioClipMap.clear();
	}
}