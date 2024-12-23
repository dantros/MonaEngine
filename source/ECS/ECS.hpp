#pragma once
#ifndef ECS_MONA_HPP
#define ECS_MONA_HPP

#include "./ComponentManager.hpp"
#include "./EventManager.hpp"
#include "./SystemManager.hpp"
#include "./Systems/BaseSystem.hpp"
#include "../World/World.hpp"
#include <entt/entt.hpp>

namespace Mona
{
    class World;
}

namespace MonaECS
{
    class ECSHandler
    {
    public:
        ECSHandler(Mona::World *world)
        {
            componentManager.SetWorld(world);
        };
        ~ECSHandler() = default;

        /*
         * Entity Management
         */
        entt::registry& GetRegistry() noexcept {return componentManager.GetRegistry();}
        entt::entity CreateEntity() noexcept {return componentManager.CreateEntity();}
        void DestroyEntity(entt::entity entity) noexcept {componentManager.DestroyEntity(entity);}
        
        /*
         * Component Management  
         */
        // Add a component to an entity
        template<typename ComponentType, typename... Args>
        ComponentType& AddComponent(entt::entity entity, Args&&... args) {return componentManager.AddComponent<ComponentType>(entity, std::forward<Args>(args)...);}
        // Get a component from an entity
        template<typename ComponentType>
        ComponentType& GetComponent(entt::entity entity) {return componentManager.GetComponent<ComponentType>(entity);}
        // Remove a component from an entity
        template<typename ComponentType>
        void RemoveComponent(entt::entity entity) {componentManager.RemoveComponent<ComponentType>(entity);}
        // Get a entt::view iterator of components with exclude types
        template<typename... ComponentTypes, typename... ExcludeTypes>
        auto ComponentView(entt::exclude_t<ExcludeTypes...> = {}) {return componentManager.ComponentQuery<ComponentTypes...>();}
        // Return a entt::view iterator of components
        template<typename... ComponentTypes>
        auto ComponentView() {return componentManager.ComponentQuery<ComponentTypes...>();}
        // Iterate over all components of a type
        template<typename... ComponentTypes, typename Func>
        void ForEachComponent(Func&& func) {componentManager.ForEach<ComponentTypes...>(std::forward<Func>(func));}
        // Get the number of components of a type
        template<typename ComponentType>
        uint32_t GetComponentCount() {return componentManager.GetComponentCount<ComponentType>();}


        /*
         * Event Management
         */
        // Subscribe to an event with a function pointer
        template<typename EventType, void(*func)(const EventType &)>
        void SubscribeEvent() {eventManager.Subscribe<EventType, func>();}
        // Subscribe to an event with a member function pointer
        template<typename EventType, typename ObjectType, void(ObjectType::*func)(const EventType &)>
        void SubscribeEvent(ObjectType& obj) {eventManager.Subscribe<EventType, ObjectType, func>(obj);}
        // Publish an event
        template<typename EventType>
        void PublishEvent(const EventType& event) {eventManager.Publish(event);}
        // Unsubscribe from an event with a function pointer
        template<typename EventType, void(*func)(const EventType &)>
        void UnsubscribeEvent() {eventManager.Unsubscribe<EventType, func>();}
        // Unsubscribe from an event with a member function pointer
        template<typename EventType, typename ObjectType, void(ObjectType::*func)(const EventType &)>
        void UnsubscribeEvent(ObjectType& obj) {eventManager.Unsubscribe<EventType, ObjectType, func>(obj);}


        /*
         * System Management
         */
        // Register a system
        template<typename SystemType, typename... Args>
        SystemType RegisterSystem(Args&&... args) {return systemManager.RegisterSystem<SystemType>(std::forward<Args>(args)...);}
        // Unregister a system
        template<typename SystemType>
        void UnregisterSystem() {systemManager.UnregisterSystem<SystemType>();}
        // Get a system
        template<typename SystemType>
        SystemType& GetSystem() {return systemManager.GetSystem<SystemType>();}
        // Start up all systems
        void StartUpSystems() {systemManager.StartUpSystems(componentManager, eventManager);}
        // Update all systems
        void UpdateSystems(float deltaTime) {systemManager.UpdateSystems(componentManager, eventManager, deltaTime);}
        // Shut down all systems
        void ShutDownSystems() {systemManager.ShutDownSystems(componentManager, eventManager);}

    private:
        ComponentManager componentManager;
        EventManager eventManager;
        SystemManager systemManager;
    };
}

#endif