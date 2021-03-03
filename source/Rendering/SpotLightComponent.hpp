#pragma once
#ifndef SPOTLIGHTCOMPONENT_HPP
#define SPOTLIGHTCOMPONENT_HPP
#include <string_view>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "../World/ComponentTypes.hpp"

namespace Mona {
	class TransformComponent;
	class SpotLightComponent {
	public:
		//using managerType = ComponentManager<SpotLightComponent>;
		using LifetimePolicyType = DefaultLifetimePolicy<SpotLightComponent>;
		using dependencies = DependencyList<TransformComponent>;
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::SpotLightComponent);
		static constexpr std::string_view componentName = "SpotLightComponent";

		static constexpr float s_epsilon = 0.01;
		SpotLightComponent(const glm::vec3& color = glm::vec3(1.0f),
			float maxRadius = 10.0f,
			float penumbraAngle = glm::radians(20.0f),
			float umbraAngle = glm::radians(25.0f),
			const glm::fquat& direction = glm::fquat(1.0f, 0.0f, 0.0f, 0.0f)) :
			m_maxRadius(maxRadius),
			m_lightColor(color),
			m_lightDirection(direction),
			m_penumbraAngle(penumbraAngle),
			m_umbraAngle(umbraAngle) {};
		const glm::vec3& GetLightColor() const { return m_lightColor; }
		void SetLightColor(const glm::vec3& color) { m_lightColor = color; }
		const glm::fquat& GetLightDirection() const { return m_lightDirection; }
		void SetLightDirection(const glm::fquat& direction) { m_lightDirection = direction; }
		float GetMaxRadius() const { return m_maxRadius; }
		void SetMaxRadius(float radius) { m_maxRadius = radius; }
		float GetUmbraAngle() const { return m_umbraAngle; }
		void SetUmbraAngle(float angle) { m_umbraAngle = angle; }
		float GetPenumbraAngle() const { return m_penumbraAngle; }
		void SetPenumbraAngle(float angle) { m_penumbraAngle = angle; }


	private:
		glm::vec3 m_lightColor;
		glm::fquat m_lightDirection;
		float m_maxRadius;
		float m_penumbraAngle;
		float m_umbraAngle;
	};
}
#endif