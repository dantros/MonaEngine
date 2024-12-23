#pragma once
#ifndef ECS_INPUT_EVENTS_HPP
#define ECS_INPUT_EVENTS_HPP
#include <glm/glm.hpp>

namespace MonaECS
{
    struct MoveInputEvent
    {
        glm::vec3 moveDir = glm::vec3(0.0f);
    };

    struct ActionInputEvent
    {
        bool ACTION1 = false;
        bool ACTION2 = false;
        bool ACTION3 = false;
    };
}

#endif