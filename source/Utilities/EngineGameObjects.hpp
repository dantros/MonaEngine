#pragma once
#ifndef ENGINEGAMEOBJECTS_HPP
#define ENGINEGAMEOBJECTS_HPP

#include "../World/GameObject.hpp"

// component definitions must be included before ComponentHandle.hpp to generate the proper handle definitions
#include "../World/TransformComponent.hpp"
#include "../Rendering/StaticMeshComponent.hpp"
#include "../World/ComponentHandle.hpp"

namespace Mona
{

class Axis : public Mona::GameObject {
    public:
        Axis() = default;
        virtual void UserStartUp(Mona::World& world) noexcept override;
    private:
        Mona::TransformHandle m_transform;
        Mona::StaticMeshHandle m_staticMesh;
    };

}

#endif