#pragma once
#ifndef POINTLIGHTCOMPONENT_HPP
#define POINTLIGHTCOMPONENT_HPP
#include <string_view>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "../World/ComponentTypes.hpp"

namespace Mona {
	class TransformComponent;
	class PointLightComponent {
	public:
		using LifetimePolicyType = DefaultLifetimePolicy<PointLightComponent>;
		using dependencies = DependencyList<TransformComponent>;
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::PointLightComponent);
		static constexpr std::string_view componentName = "PointLightComponent";
		
		static constexpr float s_epsilon = 0.01;
		PointLightComponent(const glm::vec3& color = glm::vec3(1.0f),
			float maxRadius = 10.0f,
			const glm::fquat& direction = glm::fquat(1.0f, 0.0f, 0.0f, 0.0f)) :
			m_maxRadius(maxRadius), m_lightColor(color) {};
		const glm::vec3& GetLightColor() const { return m_lightColor; }
		void SetLightColor(const glm::vec3& color) { m_lightColor = color; }
		float GetMaxRadius() const { return m_maxRadius; }
		void SetMaxRadius(float radius) { m_maxRadius = radius; }

	private:
		glm::vec3 m_lightColor;
		float m_maxRadius;
	};
}
#endif