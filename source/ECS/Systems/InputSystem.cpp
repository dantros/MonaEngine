#include "./BaseSystem.hpp"
#include "./InputSystem.hpp"
#include "../ComponentManager.hpp"
#include "../EventManager.hpp"
#include "../Events/InputEvents.hpp"
#include "../Components/InputComponents.hpp"
#include "../../Platform/Input.hpp"
#include "../../Platform/KeyCodes.hpp"

namespace MonaECS
{
    void InputSystem::Update(ComponentManager &componentManager, EventManager &eventManager, float deltaTime) noexcept
    {
        Mona::Input &minput = componentManager.GetWorld()->GetInput();
        componentManager.ForEach<MoveInputComponent>([&](entt::entity entity, MoveInputComponent &input)
                                                     { HandleMoveInput(input, minput, eventManager); });
        componentManager.ForEach<ActionInputComponent>([&](entt::entity entity, ActionInputComponent &input)
                                                       { HandleActionInput(input, minput, eventManager); });
    }

    void InputSystem::HandleMoveInput(MoveInputComponent &input, Mona::Input &minput, EventManager &eventManager)
    {
        input.FORWARD = false;
        input.BACKWARD = false;
        input.LEFT = false;
        input.RIGHT = false;
        input.UP = false;
        input.DOWN = false;

        glm::vec3 moveDir = glm::vec3(0.0f);
        if (minput.IsKeyPressed(MONA_KEY_W) or minput.IsKeyPressed(MONA_KEY_UP))
        {
            input.FORWARD = true;
            moveDir.z += 1.0f;
        }
        if (minput.IsKeyPressed(MONA_KEY_S) or minput.IsKeyPressed(MONA_KEY_DOWN))
        {
            input.BACKWARD = true;
            moveDir.z -= 1.0f;
        }
        if (minput.IsKeyPressed(MONA_KEY_A) or minput.IsKeyPressed(MONA_KEY_LEFT))
        {
            input.LEFT = true;
            moveDir.x -= 1.0f;
        }
        if (minput.IsKeyPressed(MONA_KEY_D) or minput.IsKeyPressed(MONA_KEY_RIGHT))
        {
            input.RIGHT = true;
            moveDir.x += 1.0f;
        }
        if (minput.IsKeyPressed(MONA_KEY_Q) or minput.IsKeyPressed(MONA_KEY_SPACE))
        {
            input.UP = true;
            moveDir.y += 1.0f;
        }
        if (minput.IsKeyPressed(MONA_KEY_E) or minput.IsKeyPressed(MONA_KEY_LEFT_CONTROL))
        {
            input.DOWN = true;
            moveDir.y -= 1.0f;
        }

        if (glm::length(moveDir) > 0.0f)
        {
            moveDir = glm::normalize(moveDir);
            eventManager.Publish<MoveInputEvent>({moveDir});
        }
    }

    void InputSystem::HandleActionInput(ActionInputComponent &input, Mona::Input &minput, EventManager &eventManager)
    {
        input.ACTION1 = false;
        input.ACTION2 = false;
        input.ACTION3 = false;

        if (minput.IsKeyPressed(MONA_MOUSE_BUTTON_1))
            input.ACTION1 = true;
        if (minput.IsKeyPressed(MONA_MOUSE_BUTTON_2))
            input.ACTION2 = true;
        if (minput.IsKeyPressed(MONA_KEY_ESCAPE))
            input.ACTION3 = true;

        if (input.ACTION1 || input.ACTION2 || input.ACTION3)
        {
            ActionInputEvent event{
                input.ACTION1,
                input.ACTION2,
                input.ACTION3};
            eventManager.Publish<ActionInputEvent>(event);
        }
    }

}