#pragma once
#ifndef RENDERER_HPP
#define RENDERER_HPP
#include <vector>
#include "../Event/EventManager.hpp"
#include "../World/ComponentTypes.hpp"
#include "../World/TransformComponent.hpp"
#include "StaticMeshComponent.hpp"
#include "CameraComponent.hpp"
#include "ShaderProgram.hpp"
#include "Material.hpp"
#include "../DebugDrawing/DebugDrawingSystem.hpp"



namespace Mona {


	class Renderer {
	public:
		Renderer() = default;
		void StartUp(EventManager& eventManager, DebugDrawingSystem* debugDrawingSystemPtr) noexcept;
		void Render(EventManager& eventManager,
					const InnerComponentHandle &cameraHandle,
					ComponentManager<StaticMeshComponent> &staticMeshDataManager,
					ComponentManager<TransformComponent> &transformDataManager,
					ComponentManager<CameraComponent> &cameraDataManager) noexcept;
		void ShutDown(EventManager& eventManager) noexcept;
		void OnWindowResizeEvent(const WindowResizeEvent& event);
		std::shared_ptr<Material> CreateMaterial(MaterialType type);
	private:
		std::vector<ShaderProgram> m_shaders;
		SubscriptionHandle m_onWindowResizeSubscription;
		DebugDrawingSystem* m_debugDrawingSystemPtr = nullptr;

	};
}
#endif