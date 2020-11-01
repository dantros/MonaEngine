#pragma once
#ifndef STATICMESHCOMPONENT_HPP
#define STATICMESHCOMPONENT_HPP
#include "../World/ComponentTypes.hpp"
#include "Mesh.hpp"
#include <glm/glm.hpp>
namespace Mona {
	class TransformComponent;
	class StaticMeshComponent
	{
	public:
		friend class Renderer;
		using managerType = ComponentManager<StaticMeshComponent>;
		using dependencies = DependencyList<TransformComponent>;
		static constexpr std::string_view componentName = "StaticMeshComponent";
		static constexpr uint8_t componentIndex = GetComponentIndex(EComponentType::StaticMeshComponent);
		StaticMeshComponent(std::shared_ptr<Mesh> mesh = nullptr,
			const glm::vec3 &c = glm::vec3(1.0f)) : m_meshPtr(mesh), m_color(c)
		{}
		

		const glm::vec3& GetColor() const noexcept {
			return m_color;
		}
		
		uint32_t GetMeshIndexCount() const noexcept {
			return m_meshPtr->GetCount();
		}

		uint32_t GetMeshVAOID() const noexcept {
			return m_meshPtr->GetVAOID();
		}

	private:
		std::shared_ptr<Mesh> m_meshPtr;
		glm::vec3 m_color;
	};
}
#endif