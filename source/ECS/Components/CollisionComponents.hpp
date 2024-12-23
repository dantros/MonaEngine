#pragma once
#ifndef COLLISION_COMPONENTS_HPP
#define COLLISION_COMPONENTS_HPP
#include "../../MonaEngine.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace MonaECS
{
    struct ColliderComponent
    {
        glm::vec3 size;
        bool isDynamic;
    };

    struct BodyComponent
    {
        glm::vec3 velocity;
        glm::vec3 acceleration;
        float mass;
    };

    struct TransformComponent
    {
        Mona::TransformHandle *tHandle;
        Mona::TransformHandle operator*() { return *tHandle; };
    };
    
}

#endif