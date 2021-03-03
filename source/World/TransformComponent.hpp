#pragma once
#ifndef TRANSFORMCOMPONENT_HPP
#define TRANSFORMCOMPONENT_HPP
#include <string_view>
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include "ComponentTypes.hpp"
namespace Mona {
	class TransformComponent {
	public:
		using LifetimePolicyType = DefaultLifetimePolicy<TransformComponent>;
		using dependencies = DependencyList<>;
		static constexpr std::string_view componentName = "TransformComponent";
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::TransformComponent);

		TransformComponent(const glm::vec3& translation = glm::vec3(0.0f),
			const glm::fquat& rotation = glm::fquat(1.0f, 0.0f, 0.0f, 0.0f),
			const glm::vec3& scale = glm::vec3(1.0f)) :
			localTranslation(translation),
			localRotation(rotation),
			localScale(scale) {}

		const glm::vec3& GetLocalTranslation() const {
			return localTranslation;
		}
		const glm::fquat& GetLocalRotation() const {
			return localRotation;
		}
		const glm::vec3& GetLocalScale() const {
			return localScale;
		}
		glm::mat4 GetModelMatrix() const {
			const glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), localTranslation);
			const glm::mat4 rotationMatrix = glm::toMat4(localRotation);
			const glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), localScale);

			return translationMatrix * rotationMatrix * scaleMatrix;
		}
		glm::mat4 GetViewMatrixFromTransform() const {
			const glm::vec3 up = GetUpVector();
			const glm::vec3 front = GetFrontVector();
			return glm::lookAt(localTranslation, localTranslation + front, up);
		}
		void Translate(glm::vec3 translation) {
			localTranslation += translation;
		}

		void SetTranslation(const glm::vec3 translation) {
			localTranslation = translation;
		}

		void Scale(glm::vec3 scale){
			localScale *= scale;
		}

		void SetScale(const glm::vec3& scale) {
			localScale = scale;
		}
		
		void Rotate(glm::vec3 axis, float angle){
			localRotation = glm::rotate(localRotation, angle, axis);
		}

		void SetRotation(const glm::fquat& rotation) {
			localRotation = rotation;
		}

		glm::vec3 GetUpVector() const {
			return glm::rotate(localRotation, glm::vec3(0.0f, 0.0f, 1.0f));
		}
		
		glm::vec3 GetRightVector() const {
			return glm::rotate(localRotation, glm::vec3(1.0f, 0.0f, 0.0f));
		}

		glm::vec3 GetFrontVector() const {
			return glm::rotate(localRotation, glm::vec3(0.0f, 1.0f, 0.0f));
		}

	private:
		glm::vec3 localTranslation;
		glm::fquat localRotation;
		glm::vec3 localScale;
	};



}
#endif