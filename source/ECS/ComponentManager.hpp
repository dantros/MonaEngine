#pragma once
#ifndef ECS_COMPONENT_MANAGER_HPP
#define ECS_COMPONENT_MANAGER_HPP
#include <entt/entt.hpp>
#include <unordered_map>
#include <typeindex>
#include "../World/World.hpp"

namespace MonaECS
{
    class ComponentManager
    {
    public:
        ComponentManager() = default;
        ~ComponentManager() = default;

        entt::registry& GetRegistry() noexcept
        {
            return m_registry;
        }

        entt::entity CreateEntity() noexcept
        {
            return m_registry.create();
        }

        void DestroyEntity(entt::entity entity) noexcept
        {
            m_registry.destroy(entity);
        }

        template<typename ComponentType, typename... Args>
        ComponentType& AddComponent(entt::entity entity, Args&&... args)
        {
            auto& cmp = m_registry.emplace<ComponentType>(entity, std::forward<Args>(args)...);
            if (m_componentCount.find(typeid(ComponentType)) == m_componentCount.end())
            {
                m_componentCount[typeid(ComponentType)] = 1;
            }
            else
            {
                m_componentCount[typeid(ComponentType)]++;
            }
            return cmp;
        }

        template<typename ComponentType>
        ComponentType& GetComponent(entt::entity entity)
        {
            return m_registry.get<ComponentType>(entity);
        }

        template<typename ComponentType>
        void RemoveComponent(entt::entity entity)
        {
            m_registry.remove<ComponentType>(entity);
            m_componentCount[typeid(ComponentType)]--;
        }

        template<typename... ComponentTypes, typename... ExcludeTypes>
        auto ComponentQuery(entt::exclude_t<ExcludeTypes...> exclude = entt::exclude_t<>()) noexcept
        {
            return m_registry.view<ComponentTypes...>(exclude);
        }

        template<typename... ComponentTypes, typename Func>
        void ForEach(Func&& func)
        {
            m_registry.view<ComponentTypes...>().each(std::forward<Func>(func));
        }

        template<typename ComponentType>
        uint32_t GetComponentCount()
        {
            return m_componentCount[typeid(ComponentType)];
        }

        void SetWorld(Mona::World *world)
        {
            m_world = world;
        }

        Mona::World* GetWorld()
        {
            return m_world;
        }

    private:
        entt::registry m_registry;
        std::unordered_map<std::type_index, uint32_t> m_componentCount;
        Mona::World *m_world;
    };   
}


#endif