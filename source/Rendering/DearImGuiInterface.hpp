#pragma once
#ifndef DEARIMGUIINTERFACE_HPP
#define DEARIMGUIINTERFACE_HPP

#include "../Event/EventManager.hpp"
#include "../Rendering/ShaderProgram.hpp"
#include <glm/glm.hpp>
#include <memory>

namespace Mona
{
class DearImGuiInterface
{
public:
    DearImGuiInterface() = default;
    void StartUp() noexcept;
    void Draw(EventManager& eventManager) noexcept;
    void ShutDown() noexcept;
};
}

#endif