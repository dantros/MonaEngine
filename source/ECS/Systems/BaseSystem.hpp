#pragma once
#ifndef BASE_ECS_SYSTEM_HPP
#define BASE_ECS_SYSTEM_HPP

namespace MonaECS {
    
    class ComponentManager;
    class EventManager;
    
    class BaseSystem {
    public:
        virtual ~BaseSystem() = default;
        void StartUp(ComponentManager& componentManager, EventManager& eventManager) noexcept {};
        virtual void Update(ComponentManager& componentManager, EventManager& eventManager, float deltaTime) noexcept = 0;
        void ShutDown(ComponentManager& componentManager, EventManager& eventManager) noexcept {};
    protected:
    };
}

#endif