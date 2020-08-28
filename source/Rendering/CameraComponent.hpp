#pragma once
#ifndef CAMERACOMPONENT_HPP
#define CAMERACOMPONENT_HPP
#include "../World/Component.hpp"
namespace Mona {
	class CameraComponent
	{
	public:
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::CameraComponent);
		float fieldOfView;
		float zNearPlane;
		float zFarPlane;
		glm::mat4 view, projection, viewProjection;
	};
}
#endif