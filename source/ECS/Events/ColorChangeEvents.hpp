#pragma once
#ifndef ECS_COLOR_CHANGE_EVENTS_HPP
#define ECS_COLOR_CHANGE_EVENTS_HPP
#include <entt/entt.hpp>

namespace MonaECS
{
    struct ColorChangeEvent
    {
        entt::entity entity;
    };
}

#endif