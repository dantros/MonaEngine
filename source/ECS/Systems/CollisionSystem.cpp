#include "./BaseSystem.hpp"
#include "./CollisionSystem.hpp"
#include "../ComponentManager.hpp"
#include "../EventManager.hpp"
#include "../Components/CollisionComponents.hpp"
#include "../Events/CollisionEvents.hpp"
#include "../../MonaEngine.hpp"
#include <iostream>

namespace MonaECS
{

    void CollisionSystem::Update(ComponentManager &componentManager, EventManager &eventManager, float deltaTime) noexcept
    {
        // UpdatePositions(componentManager, deltaTime);
        auto collisions = CheckCollisions(componentManager);
        for (auto &collision : collisions)
        {
            eventManager.Publish<CollisionEvent>({collision.entity1, collision.entity2, collision.normal});
        }
    }

    void CollisionSystem::UpdatePositions(ComponentManager &componentManager, float deltaTime) noexcept
    {
        componentManager.ForEach<TransformComponent, BodyComponent>(
            [&](entt::entity entity, TransformComponent &transform, BodyComponent &body)
            {
                body.velocity += body.acceleration * deltaTime;
                glm::vec3 translation = (*transform.tHandle)->GetLocalTranslation();
                translation += body.velocity * deltaTime;
                (*transform.tHandle)->SetTranslation(translation);
            });
    }

    std::vector<CollisionEvent> CollisionSystem::CheckCollisions(ComponentManager &componentManager) noexcept
    {
        std::vector<CollisionEvent> collisions;

        auto tc = componentManager.ComponentQuery<TransformComponent, ColliderComponent>();

        for (auto itA = tc.begin(); itA != tc.end(); ++itA)
        {
            entt::entity entityA = *itA;
            ColliderComponent colliderA = componentManager.GetComponent<ColliderComponent>(entityA);
            TransformComponent transformA = componentManager.GetComponent<TransformComponent>(entityA);
            for (auto itB = std::next(itA); itB != tc.end(); ++itB)
            {
                entt::entity entityB = *itB;
                ColliderComponent colliderB = componentManager.GetComponent<ColliderComponent>(entityB);
                TransformComponent transformB = componentManager.GetComponent<TransformComponent>(entityB);
                if (CheckAABB(transformA, colliderA, transformB, colliderB))
                {
                    auto normal = GetCollisionNormal(transformA, colliderA, transformB, colliderB);
                    collisions.push_back({entityA, entityB, normal});
                }
            }
        }
        return collisions;
    }

    bool CollisionSystem::CheckAABB(const TransformComponent &transform1, const ColliderComponent &collider1, const TransformComponent &transform2, const ColliderComponent &collider2) noexcept
    {
        glm::vec3 min1 = (*transform1.tHandle)->GetLocalTranslation() - collider1.size;
        glm::vec3 max1 = (*transform1.tHandle)->GetLocalTranslation() + collider1.size;
        glm::vec3 min2 = (*transform2.tHandle)->GetLocalTranslation() - collider2.size;
        glm::vec3 max2 = (*transform2.tHandle)->GetLocalTranslation() + collider2.size;

        if (max1.x < min2.x || min1.x > max2.x)
            return false;
        if (max1.y < min2.y || min1.y > max2.y)
            return false;
        if (max1.z < min2.z || min1.z > max2.z)
            return false;

        return true;
    }

    glm::vec3 CollisionSystem::GetCollisionNormal(const TransformComponent& transform1, const ColliderComponent& collider1, const TransformComponent& transform2, const ColliderComponent& collider2) noexcept
    {
        glm::vec3 pos1 = (*transform1.tHandle)->GetLocalTranslation();
        glm::vec3 pos2 = (*transform2.tHandle)->GetLocalTranslation();

        glm::vec3 min1 = pos1 - collider1.size;
        glm::vec3 max1 = pos1 + collider1.size;
        glm::vec3 min2 = pos2 - collider2.size;
        glm::vec3 max2 = pos2 + collider2.size;

        glm::vec3 normal(0.0f);

        float xOverlap = std::min(max1.x, max2.x) - std::max(min1.x, min2.x);
        float yOverlap = std::min(max1.y, max2.y) - std::max(min1.y, min2.y);
        float zOverlap = std::min(max1.z, max2.z) - std::max(min1.z, min2.z);

        if (xOverlap < yOverlap && xOverlap < zOverlap) {
            normal.x = (pos1.x < pos2.x) ? -1.0f : 1.0f;
        } else if (yOverlap < xOverlap && yOverlap < zOverlap) {
            normal.y = (pos1.y < pos2.y) ? -1.0f : 1.0f;
        } else {
            normal.z = (pos1.z < pos2.z) ? -1.0f : 1.0f;
        }

        return normal;
    }
}