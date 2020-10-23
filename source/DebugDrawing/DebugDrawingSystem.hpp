#pragma once
#ifndef DEBUGDRAWINGSYSTEM_HPP
#define DEBUGDRAWINGSYSTEM_HPP
#include "../Event/EventManager.hpp"
#include "../Rendering/ShaderProgram.hpp"
#include "BulletDebugDraw.hpp"
#include <glm/glm.hpp>
#include <memory>
#if NDEBUG
namespace Mona {
	class PhysicsCollisionSystem;
	class DebugDrawingSystem {
	public:
		DebugDrawingSystem() = default;
		void StartUp(PhysicsCollisionSystem* physicsSystemPtr)  noexcept {}
		void Draw(EventManager& eventManager, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) noexcept {}
		void ShutDown() noexcept{}
	};
}
#else

class btDynamicsWorld;
namespace Mona {
	class PhysicsCollisionSystem;
	class BulletDebugDraw;
	class DebugDrawingSystem {
	public:
		DebugDrawingSystem() = default;
		void StartUp(PhysicsCollisionSystem* physicsSystemPtr)  noexcept;
		void Draw(EventManager& eventManager, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) noexcept;
		void ShutDown() noexcept;
	private:
		btDynamicsWorld* m_physicsWorldPtr = nullptr;
		std::unique_ptr<BulletDebugDraw> m_bulletDebugDrawPtr;
		ShaderProgram m_lineShader;
	};
}
#endif

#endif