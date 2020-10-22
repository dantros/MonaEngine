#pragma once
#ifndef AUDIOCLIPMANAGER_HPP
#define AUDIOCLIPMANAGER_HPP
#include <memory>
#include <unordered_map>
#include <filesystem>
#include <string>
#include "AudioClip.hpp"
namespace Mona {
	class AudioClipManager {
	public:
		using AudioClipMap = std::unordered_map<std::string, std::shared_ptr<AudioClip>>;
		AudioClipManager() = default;
		std::shared_ptr<AudioClip> LoadAudioClip(const std::filesystem::path& filePath) noexcept;
		void CleanUnusedAudioClips() noexcept;
		void ShutDown() noexcept;
	private:
		AudioClipMap m_audioClipMap;
	};
}
#endif