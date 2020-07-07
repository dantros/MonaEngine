#pragma once
#ifndef COMPONENT_HPP
#define COMPONENT_HPP
#include <cstdint>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
namespace Mona {
	

	enum class ComponentType {
		TransformComponent,
		CameraComponent,
		StaticMeshComponent,
		ComponentCount
	};

	constexpr size_t GetComponentIndex(ComponentType type) {
		return static_cast<size_t>(type);
	}
	constexpr size_t GetComponentCount()
	{
		return static_cast<size_t>(ComponentType::ComponentCount);
	}

	class TransformComponent {
	public:
		static constexpr size_t componentIndex = GetComponentIndex(ComponentType::TransformComponent);
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

	class CameraComponent
	{
	public:
		static constexpr size_t componentIndex = GetComponentIndex(ComponentType::CameraComponent);
		float fieldOfView;
		float zNearPlane;
		float zFarPlane;
		glm::mat4 view, projection, viewProjection;
	};
	//TODO(BYRON): This is momentary
	typedef uint32_t MeshHandle;
	class StaticMeshComponent
	{
	public:
		static constexpr size_t componentIndex = GetComponentIndex(ComponentType::StaticMeshComponent);
		MeshHandle Handle;
	};
}
#endif