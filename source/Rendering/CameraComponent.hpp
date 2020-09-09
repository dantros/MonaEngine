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
		
		CameraComponent(float fov = 50.0f, 
			float ratio = 16.0f / 9.0f,
			float nearPlane = 0.1f,
			float farPlane = 100.0f) :
			fieldOfView(fov),
			aspectRatio(ratio),
			zNearPlane(nearPlane),
			zFarPlane(farPlane)
		{}

		glm::mat4 GetProjectionMatrix() const {
			return glm::perspective(glm::radians(fieldOfView), aspectRatio, zNearPlane, zFarPlane);
		}
		float fieldOfView = 50.0f;
		float zNearPlane = 0.1f;
		float zFarPlane = 100.0f;
		float aspectRatio = 16.0f / 9.0f;
	};

}
#endif