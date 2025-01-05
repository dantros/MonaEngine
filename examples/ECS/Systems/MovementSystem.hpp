#pragma once
#ifndef MOVEMENT_SYSTEM_HPP
#define MOVEMENT_SYSTEM_HPP

#include <MonaEngine.hpp>

namespace MonaECS {
    class BaseSystem;
    struct TransformComponent;
    struct BodyComponent;

    class MovementSystem : public BaseSystem
    {
    public:
        MovementSystem() = default;
        ~MovementSystem() = default;
        void Update(ComponentManager& componentManager, EventManager& eventManager, float deltaTime) noexcept;
    private:
        void HandleContinuousMovement(TransformComponent& transform, BodyComponent& body, float deltaTime);
    };
}

#endif
