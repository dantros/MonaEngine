#pragma once
#ifndef BASICCAMERACONTROLLER_HPP
#define BASICCAMERACONTROLLER_HPP
#include "../World/World.hpp"
#include "../Platform/KeyCodes.hpp"
#include <numbers>
#include <iostream>
#include "glm/gtx/rotate_vector.hpp"

namespace Mona {
    class BasicPerspectiveCamera_1 : public GameObject {
    public:
        BasicPerspectiveCamera_1() = default;
        virtual void UserStartUp(World& world) noexcept override
        {
			m_transform = world.AddComponent<TransformComponent>(*this);
			m_camera = world.AddComponent<CameraComponent>(*this);
			m_transform->Translate(glm::vec3(0.0f, -100.0f, 0.0f));
			auto& input = world.GetInput();
			glm::vec2 res = world.GetWindow().GetWindowDimensions();
			screenPos = glm::vec2(1 / res.x, 1 / res.y) * glm::vec2(input.GetMousePosition());
        }
		void SetActive(bool active) { m_active = active; }
        virtual void UserUpdate(World& world, float timeStep) noexcept override
        {
			auto& input = world.GetInput();
			if(m_active) {
            
			if (input.IsKeyPressed(MONA_KEY_A)) {
				glm::vec3 right = m_transform->GetRightVector();
				m_transform->Translate(-m_cameraSpeed * timeStep * right);
			}
			else if (input.IsKeyPressed(MONA_KEY_D)) {
				glm::vec3 right = m_transform->GetRightVector();
				m_transform->Translate(m_cameraSpeed * timeStep * right);
			}

			if (input.IsKeyPressed(MONA_KEY_W)) {
				glm::vec3 front = m_transform->GetFrontVector();
				m_transform->Translate(m_cameraSpeed * timeStep * front);
			}
			else if (input.IsKeyPressed(MONA_KEY_S)) {
				glm::vec3 front = m_transform->GetFrontVector();
				m_transform->Translate(-m_cameraSpeed * timeStep * front);
			}

			if (input.IsKeyPressed(MONA_KEY_E)) {
				m_transform->Rotate(glm::vec3(0.0f,1.0f,0.0f), m_rollSpeed * timeStep);
			}
			else if (input.IsKeyPressed(MONA_KEY_Q)) {
				m_transform->Rotate(glm::vec3(0.0f, 1.0f, 0.0f), -m_rollSpeed * timeStep);
			}
			}
			
			glm::vec2 res = world.GetWindow().GetWindowDimensions();
			glm::vec2 newScreenPos = glm::vec2(1/res.x, 1/res.y) * glm::vec2(input.GetMousePosition());
			glm::vec2 delta = newScreenPos - screenPos;
			if (glm::length2(delta) != 0.0f && m_active)
			{
				float amountX = delta.x * m_rotationSpeed;
				float amountY = delta.y * m_rotationSpeed;
				m_transform->Rotate(glm::vec3(0.0f,0.0f,-1.0f), amountX);
				m_transform->Rotate(glm::vec3(-1.0, 0.0f, 0.0f), amountY);
			}
			screenPos = newScreenPos;
		
        }
    private:
		bool m_active = true;
		float m_cameraSpeed = 2.0f;
		float m_rollSpeed = 1.5f;
		float m_rotationSpeed = 1.5f;
		TransformHandle m_transform;
		CameraHandle m_camera;
		glm::vec2 screenPos;
  };


	class BasicPerspectiveCamera_2 : public GameObject {
	public:
		BasicPerspectiveCamera_2() = default;
		virtual void UserStartUp(World& world) noexcept override
		{
			m_transform = world.AddComponent<TransformComponent>(*this);
			m_camera = world.AddComponent<CameraComponent>(*this);
			m_camera->SetZFarPlane(1000);
			m_transform->SetTranslation(glm::vec3(0.0f, -80.0f, 10.0f));
			auto& input = world.GetInput();
			glm::vec2 res = world.GetWindow().GetWindowDimensions();
		}
		void SetActive(bool active) { m_active = active; }
		virtual void UserUpdate(World& world, float timeStep) noexcept override
		{
			if (m_active) {
				auto& input = world.GetInput();
				glm::vec2 res = world.GetWindow().GetWindowDimensions();
				glm::vec2 screenPos = glm::vec2(1 / res.x, 1 / res.y) * glm::vec2(input.GetMousePosition());
				float maxVerticalRotationAngle = 3 * std::numbers::pi / 8;
				float maxLateralRotationAngle = 3 * std::numbers::pi / 8;

				m_transform->SetRotation(glm::identity<glm::fquat>());

				float verticalRotFactor = (1 - screenPos[1] - 0.5) / 0.5;
				float verticalRotSign = verticalRotFactor == 0 ? 1 : verticalRotFactor / std::abs(verticalRotFactor);
				float currVerticalRotAngle = maxVerticalRotationAngle * verticalRotFactor;
				m_transform->Rotate({1,0,0}, currVerticalRotAngle);

				float lateralRotFactor = -(screenPos[0] - 0.5) / 0.5;
				float currLateralRotAngle = m_lateralRotationCenter + maxLateralRotationAngle * lateralRotFactor;
				m_transform->Rotate({ 0,0,1 }, currLateralRotAngle);
				if (0.95 < abs(lateralRotFactor)) {
					// rotacion adicional
					float lateralRotSign = lateralRotFactor / std::abs(lateralRotFactor);
					float additionalRotAngle = lateralRotSign * m_rotationSpeed * timeStep;
					m_transform->Rotate({ 0,0,1 }, additionalRotAngle);
					m_lateralRotationCenter += additionalRotAngle;
				}

				float currScrollOffset = input.GetMouseWheelOffset()[1];
				m_cameraSpeed += currScrollOffset;
				m_cameraSpeed = 0 <= m_cameraSpeed ? m_cameraSpeed : 0;

				if (input.IsKeyPressed(MONA_KEY_A)) {
					glm::vec3 right = glm::rotateZ(glm::vec3(1, 0 ,0), m_lateralRotationCenter);
					m_transform->Translate(-m_cameraSpeed * timeStep * right);
				}
				else if (input.IsKeyPressed(MONA_KEY_D)) {
					glm::vec3 right = glm::rotateZ(glm::vec3(1, 0, 0), m_lateralRotationCenter);
					m_transform->Translate(m_cameraSpeed * timeStep * right);
				}

				if (input.IsKeyPressed(MONA_KEY_W)) {
					glm::vec3 front = glm::rotateZ(glm::vec3(0, 1, 0), m_lateralRotationCenter);
					m_transform->Translate(m_cameraSpeed * timeStep * front);
				}
				else if (input.IsKeyPressed(MONA_KEY_S)) {
					glm::vec3 front = glm::rotateZ(glm::vec3(0, 1, 0), m_lateralRotationCenter);
					m_transform->Translate(-m_cameraSpeed * timeStep * front);
				}

				if (input.IsKeyPressed(MONA_KEY_E)) {
					m_transform->Translate(-m_cameraSpeed * timeStep* glm::vec3({ 0,0,1 }));
				}
				else if (input.IsKeyPressed(MONA_KEY_Q)) {
					m_transform->Translate(m_cameraSpeed * timeStep * glm::vec3({ 0,0,1 }));
				}
			}

		}
	private:
		bool m_active = true;
		float m_cameraSpeed = 2.0f;
		float m_rotationSpeed = 1.5f;
		float m_lateralRotationCenter = 0;
		TransformHandle m_transform;
		CameraHandle m_camera;
	};
}











#endif