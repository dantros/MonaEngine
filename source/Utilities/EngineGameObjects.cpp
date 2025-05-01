
#include "EngineGameObjects.hpp"
#include "../Core/Config.hpp"
#include "../Rendering/MeshManager.hpp"
#include "../World/ComponentManager.hpp"
#include "../World/World.hpp"
#include "../Rendering/TextureManager.hpp"
#include "../Rendering/UnlitTexturedMaterial.hpp"

namespace Mona
{

void Axis::UserStartUp(Mona::World& world) noexcept
{
    auto& config = Mona::Config::GetInstance();
    auto& meshManager = Mona::MeshManager::GetInstance();
    auto& textureManager = Mona::TextureManager::GetInstance();

    auto meshPtr = meshManager.LoadMesh(Mona::Mesh::PrimitiveType::Axis);

    auto material = world.CreateMaterial(Mona::MaterialType::UnlitTextured);
    auto materialPtr = std::static_pointer_cast<Mona::UnlitTexturedMaterial>(material);

    auto texture = textureManager.LoadTexture(config.getPathOfEngineAsset("Textures/aek-32.png"));
    materialPtr->SetUnlitColorTexture(texture);

    // siempre necesitamos una transform component para los static mesh
    m_transform = world.AddComponent<Mona::TransformComponent>(*this);

    m_staticMesh = world.AddComponent<Mona::StaticMeshComponent>(*this, meshPtr, materialPtr);
}

}