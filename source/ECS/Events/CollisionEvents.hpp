#pragma once
#ifndef ECS_COLLISION_EVENTS_HPP
#define ECS_COLLISION_EVENTS_HPP
#include <entt/entt.hpp>
#include <glm/glm.hpp>

namespace MonaECS
{
    struct CollisionEvent
    {
        entt::entity entity1;
        entt::entity entity2;
        glm::vec3 normal;
    };
}

#endif