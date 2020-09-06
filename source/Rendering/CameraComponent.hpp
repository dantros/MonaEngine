#pragma once
#ifndef CAMERACOMPONENT_HPP
#define CAMERACOMPONENT_HPP
#include "../World/Component.hpp"
#include "../Core/Log.hpp"
namespace Mona {
	class CameraComponent
	{
	public:
		using dependencies = DependencyList<TransformComponent>;
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::CameraComponent);
		static constexpr std::string_view componentName = "CameraComponent";
		
		CameraComponent() {
			tanHalfFov = glm::tan(glm::radians(fieldOfView / 2.0f));
		}
		float zNearPlane = 0.1f;
		float zFarPlane = 100.0f;
		float aspectRatio = 16.0f / 9.0f;
		glm::mat4 GetProjectionMatrix() const {
			return glm::perspective(glm::radians(fieldOfView), aspectRatio, zNearPlane, zFarPlane);
		}
		glm::vec3 ScreenToCameraSpace(const glm::vec2& screenPos, const glm::vec2& screenDimensions) const {
			float x = tanHalfFov * zNearPlane * aspectRatio * ((2 * screenPos.x / screenDimensions.x - 1));
			float y = tanHalfFov * zNearPlane * ((2 * screenPos.y / screenDimensions.y - 1));
			float z = zNearPlane;
			return glm::vec3(x, z, y);
		}
	private:
		float fieldOfView = 50.0f;
		float tanHalfFov;
		void SetFOV(float fovy) {
			fieldOfView = fovy;
			tanHalfFov = glm::tan(glm::radians(fieldOfView / 2.0f));
		}

	};

}
#endif