#pragma once
#ifndef IKNAVIGATIONCOMPONENT_HPP
#define IKNAVIGATIONCOMPONENT_HPP

#include <string_view>
#include "../World/ComponentTypes.hpp"
#include "EnvironmentData.hpp"
namespace Mona {
	class IKNavigationLifetimePolicy;
	class IKNavigationComponent{
		public:
			friend class World;
			using LifetimePolicyType = IKNavigationLifetimePolicy;
			using dependencies = DependencyList<SkeletalMeshComponent, RigidBodyComponent>;
			static constexpr std::string_view componentName = "IKNavigationComponent";
			static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::IKNavigationComponent);
			IKNavigationComponent() = default;
		private:
			EnvironmentData environmentData;
	};
}






#endif