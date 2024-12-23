#pragma once
#ifndef ECS_COLLISION_SYSTEM_HPP
#define ECS_COLLISION_SYSTEM_HPP
#include <vector>
#include <glm/glm.hpp>

namespace MonaECS
{
    class BaseSystem;
    struct TransformComponent;
    struct ColliderComponent;
    struct CollisionEvent;

    class CollisionSystem : public BaseSystem
    {
    public:
        CollisionSystem() = default;
        ~CollisionSystem() = default;
        void Update(ComponentManager& componentManager, EventManager& eventManager, float deltaTime) noexcept;
    private:
        void UpdatePositions(ComponentManager& componentManager, float deltaTime) noexcept;
        std::vector<CollisionEvent> CheckCollisions(ComponentManager& componentManager) noexcept;
        bool CheckAABB(const TransformComponent& transform1, const ColliderComponent& collider1, const TransformComponent& transform2, const ColliderComponent& collider2) noexcept;
        glm::vec3 GetCollisionNormal(const TransformComponent& transform1, const ColliderComponent& collider1, const TransformComponent& transform2, const ColliderComponent& collider2) noexcept;
    };
}

#endif