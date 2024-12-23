#pragma once
#ifndef INPUT_SYSTEM_HPP
#define INPUT_SYSTEM_HPP

namespace Mona
{
    class Input;
}

namespace MonaECS
{
    class BaseSystem;
    struct MoveInputComponent;
    struct ActionInputComponent;

    class InputSystem : public BaseSystem
    {
    public:
        InputSystem() = default;
        ~InputSystem() = default;
        void Update(ComponentManager &componentManager, EventManager &eventManager, float deltaTime) noexcept;
    private:
        void HandleMoveInput(MoveInputComponent &input, Mona::Input& minput, EventManager &eventManager);
        void HandleActionInput(ActionInputComponent &input, Mona::Input& minput, EventManager &eventManager);
    };
}

#endif