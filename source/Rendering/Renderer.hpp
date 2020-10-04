#pragma once
#ifndef RENDERER_HPP
#define RENDERER_HPP
#include "../Event/EventManager.hpp"
#include "../World/ComponentManager.hpp"
#include "../World/ComponentTypes.hpp"
#include "../World/TransformComponent.hpp"
#include "StaticMeshComponent.hpp"
#include "CameraComponent.hpp"
#include "ShaderProgram.hpp"
#include "../DebugDrawing/DebugDrawingSystem.hpp"



namespace Mona {


	class Renderer {
	public:
		Renderer() = default;
		void StartUp(EventManager& eventManager, DebugDrawingSystem* debugDrawingSystemPtr) noexcept;
		void Render(EventManager& eventManager,
					InnerComponentHandle cameraHandle,
					ComponentManager<StaticMeshComponent> &staticMeshDataManager,
					ComponentManager<TransformComponent> &transformDataManager,
					ComponentManager<CameraComponent> &cameraDataManager) noexcept;
		void ShutDown(EventManager& eventManager) noexcept;
		void OnWindowResizeEvent(const WindowResizeEvent& event);
	private:
		ShaderProgram m_shader;
		SubscriptionHandle m_onWindowResizeSubscription;
		DebugDrawingSystem* m_debugDrawingSystemPtr = nullptr;

	};
}
#endif