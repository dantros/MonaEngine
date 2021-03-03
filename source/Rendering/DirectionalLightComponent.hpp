#pragma once
#ifndef DIRECTIONALLIGHTCOMPONENT_HPP
#define DIRECTIONALLIGHTCOMPONENT_HPP
#include <string_view>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "../World/ComponentTypes.hpp"

namespace Mona {
	class TransformComponent;
	class DirectionalLightComponent {
	public:
		using LifetimePolicyType = DefaultLifetimePolicy<DirectionalLightComponent>;
		using dependencies = DependencyList<TransformComponent>;
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::DirectionalLightComponent);
		static constexpr std::string_view componentName = "DirectionalLightComponent";

		DirectionalLightComponent(const glm::vec3& color = glm::vec3(1.0f),
			const glm::fquat& direction = glm::fquat(1.0f, 0.0f, 0.0f, 0.0f)) :
			m_lightColor(color), m_lightDirection(direction) {};
		const glm::vec3& GetLightColor() const { return m_lightColor; }
		void SetLightColor(const glm::vec3& color) { m_lightColor = color; }
		const glm::fquat& GetLightDirection() const { return m_lightDirection; }
		void SetLightDirection(const glm::fquat& direction) { m_lightDirection = direction; }

	private:
		glm::vec3 m_lightColor;
		glm::fquat m_lightDirection;
	};
}
#endif