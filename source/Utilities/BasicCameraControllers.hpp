#pragma once
#ifndef BASICCAMERACONTROLLER_HPP
#define BASICCAMERACONTROLLER_HPP
#include "../World/World.hpp"
#include "../Platform/KeyCodes.hpp"
namespace Mona {
    class BasicPerspectiveCamera : public GameObject {
    public:
        BasicPerspectiveCamera() = default;
        virtual void UserStartUp(World& world) noexcept override
        {
			m_transform = world.AddComponent<TransformComponent>(*this);
			m_camera = world.AddComponent<CameraComponent>(*this);
			m_transform->Translate(glm::vec3(0.0f, -5.0f, 0.0f));
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
}











#endif