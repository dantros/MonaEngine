#include "./BaseSystem.hpp"
#include "./MovementSystem.hpp"
#include "../ComponentManager.hpp"
#include "../EventManager.hpp"
#include "../Components/InputComponents.hpp"
#include "../Components/CollisionComponents.hpp"
#include "../Events./InputEvents.hpp"


namespace MonaECS
{
    void MovementSystem::Update(ComponentManager &componentManager, EventManager &eventManager, float deltaTime) noexcept
    {
        auto view = componentManager.ComponentQuery<TransformComponent, BodyComponent>(entt::exclude_t<MoveInputComponent>());
        view.each([&](auto entity, TransformComponent &transform, BodyComponent &body) {
            HandleContinuousMovement(transform, body, deltaTime);
        });
    }

    void MovementSystem::HandleContinuousMovement(TransformComponent &transform, BodyComponent &body, float deltaTime)
    {
        body.velocity += body.acceleration * deltaTime;
        glm::vec3 translation = (*transform.tHandle)->GetLocalTranslation();
        translation += body.velocity * deltaTime;
        (*transform.tHandle)->SetTranslation(translation);
    }
} 