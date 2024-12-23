#include "./BaseSystem.hpp"
#include "./StatsSystem.hpp"
#include "../Components.hpp"
#include "../ComponentManager.hpp"
#include "../EventManager.hpp"
#include "../Events.hpp"
#include <iostream>

namespace MonaECS
{

    void StatsSystem::Update(ComponentManager &componentManager, EventManager &eventManager, float deltaTime) noexcept
    {
        s_time += deltaTime;
        if (s_time >= 1.0f)
        {
            s_time = 0.0f;
            std::cout << "StatsSystem: Update" << std::endl;
        }

        auto vw = componentManager.ComponentQuery<Stats>();

        componentManager.ForEach<Stats>(
            [&](entt::entity entity, Stats &stats)
            {
                if (stats.health > 30.0f && stats.health <= 60.0f)
                {
                    stats.state = StatsColors::green;
                    eventManager.Publish<ColorChangeEvent>({entity});
                }
                else if (stats.health > 0.0f && stats.health <= 30.0f)
                {
                    stats.state = StatsColors::blue;
                    eventManager.Publish<ColorChangeEvent>({entity});
                }
                else if (stats.health <= 0.0f)
                {
                    eventManager.Publish<DestroyGameObjectEvent>({entity});
                }
                else
                {
                    stats.state = StatsColors::red;
                }
            });
    }
}