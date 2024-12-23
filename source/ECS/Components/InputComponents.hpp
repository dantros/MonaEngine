#pragma once
#ifndef ECS_INPUT_COMPONENTS_HPP
#define ECS_INPUT_COMPONENTS_HPP

namespace MonaECS
{
    
    struct MoveInputComponent
    {
        bool FORWARD = false;
        bool BACKWARD = false;
        bool LEFT = false;
        bool RIGHT = false;
        bool UP = false;
        bool DOWN = false;
    };

    struct ActionInputComponent
    {
        bool ACTION1 = false;
        bool ACTION2 = false;
        bool ACTION3 = false;
    };

}

#endif