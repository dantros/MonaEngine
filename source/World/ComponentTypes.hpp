#pragma once
#ifndef COMPONENTTYPES_HPP
#define COMPONENTTYPES_HPP
#include <cstdint>
#include "../Core/Common.hpp"
namespace Mona {
	enum class EComponentType : uint8_t {
		TransformComponent,
		CameraComponent,
		StaticMeshComponent,
		RigidBodyComponent,
		AudioSourceComponent,
		DirectionalLightComponent,
		SpotLightComponent,
		PointLightComponent,
		SkeletalMeshComponent,
		ComponentTypeCount
	};

	constexpr uint8_t GetComponentIndex(EComponentType type) {
		return static_cast<uint8_t>(type);
	}
	constexpr uint8_t GetComponentTypeCount()
	{
		return static_cast<uint8_t>(EComponentType::ComponentTypeCount);
	}
	template <typename ...ComponentTypes> struct DependencyList {};

	class TransformComponent;
	class StaticMeshComponent;
	class CameraComponent;
	class RigidBodyComponent;
	class AudioSourceComponent;
	class DirectionalLightComponent;
	class SpotLightComponent;
	class PointLightComponent;
	class SkeletalMeshComponent;
	template <typename ComponentType>
	class DefaultLifetimePolicy;


	template <typename ComponentType>
		class ComponentManager;

	template <typename ComponentType>
	inline constexpr bool is_component = is_any<ComponentType,
		TransformComponent,
		CameraComponent,
		StaticMeshComponent,
		RigidBodyComponent,
		AudioSourceComponent,
		DirectionalLightComponent,
		SpotLightComponent,
		PointLightComponent,
		SkeletalMeshComponent>;

}
#endif