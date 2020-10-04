#pragma once
#ifndef STATICMESHCOMPONENT_HPP
#define STATICMESHCOMPONENT_HPP
#include "../World/ComponentTypes.hpp"
#include "ModelManager.hpp"
#include <glm/glm.hpp>
namespace Mona {
	class TransformComponent;
	class StaticMeshComponent
	{
	public:
		using managerType = ComponentManager<StaticMeshComponent>;
		using dependencies = DependencyList<TransformComponent>;
		static constexpr std::string_view componentName = "StaticMeshComponent";
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::StaticMeshComponent);
		StaticMeshComponent(ModelManager::PrimitiveType type = ModelManager::PrimitiveType::Cube,
			const glm::vec3 &c = glm::vec3(1.0f)) : handle(ModelManager::GetInstance().LoadModel(type)), color(c)
		{}
		ModelManager::ModelHandle& GetHandle() noexcept{
			return handle;
		}
		const ModelManager::ModelHandle& GetHandle() const noexcept{
			return handle;
		}

		const glm::vec3& GetColor() const noexcept {
			return color;
		}

	private:
		ModelManager::ModelHandle handle;
		glm::vec3 color;
	};
}
#endif