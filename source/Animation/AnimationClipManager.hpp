#pragma once
#ifndef ANIMATIONCLIPMANAGER_HPP
#define ANIMATIONCLIPMANAGER_HPP
#include <memory>
#include <filesystem>
#include <unordered_map>
namespace Mona {
	class AnimationClip;
	class Skeleton;
	class AnimationClipManager {
		friend class World;
	public:
		using AnimationClipMap = std::unordered_map<std::string, std::shared_ptr<AnimationClip>>;
		AnimationClipManager(AnimationClipManager const&) = delete;
		AnimationClipManager& operator=(AnimationClipManager const&) = delete;
		std::shared_ptr<AnimationClip> LoadAnimationClip(const std::filesystem::path& filePath,
			std::shared_ptr<Skeleton> skeleton,
			bool removeRootMotion = true) noexcept;
		void CleanUnusedAnimationClips() noexcept;
		static AnimationClipManager& GetInstance() noexcept {
			static AnimationClipManager manager;
			return manager;
		}
	private:
		AnimationClipManager() = default;
		void ShutDown() noexcept;
		AnimationClipMap m_animationClipMap;
	};
}
#endif