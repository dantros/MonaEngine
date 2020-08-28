#pragma once
#ifndef COMPONENT_HPP
#define COMPONENT_HPP
#include <cstdint>
#include "../Core/Common.hpp"
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

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
		const glm::vec3& GetLocalTranslation() const {
			return localTranslation;
		}

		const glm::mat4& GetModelMatrix() const {
			return modelMatrix;
		}
		void Translate(glm::vec3 translation) {
			localTranslation += translation;
			UpdateModelMatrix();
		}
		void Scale(glm::vec3 scale){
			localScale += scale;
			UpdateModelMatrix();
		}
		
		void Rotate(glm::vec3 axis, float angle){
			localRotation = glm::rotate(localRotation, angle, axis);
			UpdateModelMatrix();
		}
		

	private:
		void UpdateModelMatrix()
		{
			glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), localTranslation);
			glm::mat4 rotationMatrix = glm::toMat4(localRotation);
			glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), localScale);

			modelMatrix = translationMatrix*rotationMatrix*scaleMatrix;
		}
		glm::vec3 localTranslation = glm::vec3(0.0f);
		glm::fquat localRotation = glm::fquat(0.0, 0.0, 0.0, 1.0);
		glm::vec3 localScale = glm::vec3(1.0f);
		glm::mat4 modelMatrix = glm::mat4(1.0);
		
	};

	template <typename ComponentType>
	inline constexpr bool is_component = is_any<ComponentType, TransformComponent, CameraComponent, StaticMeshComponent>;

}
#endif