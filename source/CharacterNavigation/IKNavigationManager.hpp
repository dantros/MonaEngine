#pragma once
#ifndef IKNAVIGATIONMANAGER_HPP
#define IKNAVIGATIONMANAGER_HPP
#include <string>
#include <memory>
namespace Mona {
	class IKNavigationManager {
		friend class World;
	public:
		IKNavigationManager(IKNavigationManager const&) = delete;
		IKNavigationManager& operator=(IKNavigationManager const&) = delete;
		void CleanUnusedSkeletons() noexcept;
		void AddTerrain();
		void AddAnimation(std::shared_ptr<AnimationClip> animationClip, std::shared_ptr<Skeleton> character);
		static IKNavigationManager& GetInstance() noexcept {
			static IKNavigationManager manager;
			return manager;
		}
	private:
		IKNavigationManager() = default;
		void ShutDown() noexcept;
	};
}
#endif