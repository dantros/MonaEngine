#pragma once
#ifndef IKNAVIGATIONCOMPONENT_HPP
#define IKNAVIGATIONCOMPONENT_HPP


#include <string_view>
#include <memory>
#include <vector>
#include "../Core/Log.hpp"
#include "../World/ComponentTypes.hpp"
namespace Mona {
	class Skeleton;
	class AnimationClip;

	class IKNavigationComponent{
		public:
			friend class World;
			using LifetimePolicyType = DefaultLifetimePolicy<IKNavigationComponent>;
			using dependencies = DependencyList<SkeletalMeshComponent>;
			static constexpr std::string_view componentName = "IKNavigationComponent";
			static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::IKNavigationComponent);


	};
}






#endif