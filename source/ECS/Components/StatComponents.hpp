#pragma once
#ifndef STAT_COMPONENTS_HPP
#define STAT_COMPONENTS_HPP
#include "../../MonaEngine.hpp"
#include <glm/glm.hpp>

namespace MonaECS
{
    enum class StatsColors
    {
        red,
        green,
        blue
    };
    struct Stats
    {
        float health = 100.0f;
        StatsColors state = StatsColors::red;
        glm::vec3 GetColor()
        {
            switch (this->state)
            {
            case StatsColors::red:
                return glm::vec3(1.0f, 0.0f, 0.0f);
            case StatsColors::green:
                return glm::vec3(0.0f, 1.0f, 0.0f);
            case StatsColors::blue:
                return glm::vec3(0.0f, 0.0f, 1.0f);
            default:
                return glm::vec3(1.0f, 1.0f, 1.0f);
            }
        }
    };
}

#endif