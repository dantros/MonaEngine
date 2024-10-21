#pragma once
#ifndef ENGINE_HPP
#define ENGINE_HPP
#include "Application.hpp"
#include "World/World.hpp"
#include <memory>
namespace Mona {
class Engine {
public:
    Engine(Application& app)
        : m_world(app)
    {
    }
    ~Engine() = default;
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;
    /*
     * Funci�n que comienza el main loop del motor.
     */
    void StartMainLoop() noexcept
    {
        m_world.StartMainLoop();
    }

private:
    World m_world;
};
}

#endif