#pragma once
#ifndef COMPONENTHANDLE_HPP
#define COMPONENTHANDLE_HPP
#include "GameObjectTypes.hpp"
#include "ComponentTypes.hpp"
#include "ComponentManager.hpp"
namespace Mona {
	class BaseComponentHandle {
	public:
		BaseComponentHandle() : m_innerHandle() {}
		BaseComponentHandle(InnerComponentHandle handle) : m_innerHandle(handle) {}
		InnerComponentHandle GetInnerHandle() const noexcept { return m_innerHandle; }
	protected:
		InnerComponentHandle m_innerHandle;
	};

	template <typename ComponentType>
	class ComponentHandle : public BaseComponentHandle {
		static_assert(is_component<ComponentType>, "Cannot create handle of type that isn't a component");
	public:
		static constexpr uint8_t componentIndex = ComponentType::componentIndex;
		ComponentHandle() : BaseComponentHandle(), m_managerPointer(nullptr) {}
		ComponentHandle(InnerComponentHandle handle, ComponentManager<ComponentType>* manager) : BaseComponentHandle(handle), m_managerPointer(manager) {}
		InnerComponentHandle GetInnerHandle() const noexcept { return m_innerHandle; }
		bool IsValid() const noexcept {
			if (m_managerPointer == nullptr)
				return false;
			return m_managerPointer->IsValid(m_innerHandle);
		}
		ComponentType* operator->()
		{
			MONA_ASSERT(IsValid(), "ComponentHandle Error: Trying to access with invalid componentHandle");
			return m_managerPointer->GetComponentPointer(m_innerHandle);
		}
		const ComponentType* operator->() const {
			MONA_ASSERT(IsValid(), "ComponentHandle Error: Trying to access with invalid componentHandle");
			return m_managerPointer->GetComponentPointer(m_innerHandle);
		}
	private:
		ComponentManager<ComponentType>* m_managerPointer;
	};
	using TransformHandle = ComponentHandle<TransformComponent>;
	using StaticMeshHandle = ComponentHandle<StaticMeshComponent>;
	using CameraHandle = ComponentHandle<CameraComponent>;
	using RigidBodyHandle = ComponentHandle<RigidBodyComponent>;
	using AudioSourceHandle = ComponentHandle<AudioSourceComponent>;
	using DirectionalLightHandle = ComponentHandle<DirectionalLightComponent>;
	using SpotLightHandle = ComponentHandle<SpotLightComponent>;
	using PointLightHandle = ComponentHandle<PointLightComponent>;
	using SkeletalMeshHandle = ComponentHandle<SkeletalMeshComponent>;



}
#endif