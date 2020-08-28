#pragma once
#ifndef COMPONENT_HPP
#define COMPONENT_HPP
#include <cstdint>
#include "../Core/Common.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace Mona {
	
	enum class EComponentType : uint8_t {
		TransformComponent,
		CameraComponent,
		StaticMeshComponent,
		ComponentTypeCount
	};

	constexpr uint8_t GetComponentIndex(EComponentType type) {
		return static_cast<uint8_t>(type);
	}
	constexpr uint8_t GetComponentTypeCount()
	{
		return static_cast<uint8_t>(EComponentType::ComponentTypeCount);
	}

	class TransformComponent;
	class StaticMeshComponent;
	class CameraComponent;

	class TransformComponent {
	public:
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::TransformComponent);
		void Translate(glm::vec3 translation) {
			localTranslation += translation;
			UpdateWorldTransform();
		}
		glm::vec3 GetLocalTranslation() const
		{
			return localTranslation;
		}
		glm::fquat GetLocalRotation() const {
			return localTranslation;
		}
		glm::vec3 GetLocalScale() const {
			return localScale;
		}
		glm::mat4 GetWorldTransform() const
		{
			return worldTransform;
		}

		void UpdateWorldTransform() {
			return;
		}
	private:
		glm::vec3 localTranslation = glm::vec3(0.0f);
		glm::fquat localRotation = glm::fquat(0.0, 0.0, 0.0, 1.0);
		glm::vec3 localScale = glm::vec3(1.0f);
		glm::mat4 worldTransform = glm::mat4(1.0);
		
	};

	template <typename ComponentType>
	inline constexpr bool is_component = is_any<ComponentType, TransformComponent, CameraComponent, StaticMeshComponent>;

}
#endif