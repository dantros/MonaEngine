#pragma once
#ifndef BASICCAMERACONTROLLER_HPP
#define BASICCAMERACONTROLLER_HPP
#include "../World/World.hpp"
namespace Mona {
  class BasicPerspectiveCamera : public GameObject {
    public:
    BasicPerspectiveCamera() = default;
    virtual void UserStartUp(World& world) noexcept override{
      m_transform = world.AddComponent<TransformComponent>(*this);
      m_camera = world.AddComponent<CameraComponent>(*this);

    }
    virtual void UserUpdate(World& world, float timeStep) noexcept override{
      auto& input = world.GetInput();
      if(input.IsKeyPressed(MONA_KEY_A)){
        m_transform->Translate(glm::vec3(-m_cameraSpeed*timeStep,0.0f,0.0f));
      }
      else if(input.IsKeyPressed(MONA_KEY_D)){
        m_transform->Translate(glm::vec3(m_cameraSpeed*timeStep,0.0f,0.0f));
      }

      if(input.IsKeyPressed(MONA_KEY_W)){
        m_transform->Translate(glm::vec3(0.0f,m_cameraSpeed*timeStep,0.0f));
      }
      else if(input.IsKeyPressed(MONA_KEY_S)){
        m_transform->Translate(glm::vec3(0.0f,-m_cameraSpeed*timeStep,0.0f));
      }
    }
    private:
      float m_cameraSpeed = 1.0f;
      TransformHandle m_transform;
      CameraHandle m_camera;
  };
}











#endif