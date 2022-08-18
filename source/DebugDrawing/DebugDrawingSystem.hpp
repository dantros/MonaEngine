#pragma once
#ifndef DEBUGDRAWINGSYSTEM_HPP
#define DEBUGDRAWINGSYSTEM_HPP
#include "../Event/EventManager.hpp"
#include "../Rendering/ShaderProgram.hpp"
#include "BulletDebugDraw.hpp"
#include "IKNavigationDebugDraw.hpp"
#include "../CharacterNavigation/IKNavigationSystem.hpp"
#include <glm/glm.hpp>
#include <memory>

namespace Mona {
	class DebugDrawingSystem {
	public:
		DebugDrawingSystem() = default;
		virtual void StartUp(PhysicsCollisionSystem* physicsSystemPtr)  noexcept {};
		virtual void StartUp(IKNavigationSystem* ikNavSystemPtr)  noexcept {};
		virtual void Draw(EventManager& eventManager, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) noexcept {};
		virtual void ShutDown() noexcept {};
	};
}

#if NDEBUG
namespace Mona {
	class PhysicsCollisionSystem;
	class DebugDrawingSystem_physics : public DebugDrawingSystem {
	public:
		DebugDrawingSystem_physics() = default;
		void StartUp(PhysicsCollisionSystem* physicsSystemPtr)  noexcept {}
		void Draw(EventManager& eventManager, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) noexcept {}
		void ShutDown() noexcept{}
	};
	class IKNavigationSystem;
	class DebugDrawingSystem_ikNav : public DebugDrawingSystem {
	public:
		DebugDrawingSystem_ikNav() = default;
		void StartUp(IKNavigationSystem* ikNavSystemPtr)  noexcept {}
		void Draw(EventManager& eventManager, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) noexcept {}
		void ShutDown() noexcept {}
	};
}
#else

class btDynamicsWorld;
namespace Mona {
	class PhysicsCollisionSystem;
	class BulletDebugDraw;
	class DebugDrawingSystem_physics : public DebugDrawingSystem {
	public:
		DebugDrawingSystem_physics() = default;
		void StartUp(PhysicsCollisionSystem* physicsSystemPtr)  noexcept override;
		void Draw(EventManager& eventManager, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) noexcept override;
		void ShutDown() noexcept override;
	private:
		btDynamicsWorld* m_physicsWorldPtr = nullptr;
		std::unique_ptr<BulletDebugDraw> m_bulletDebugDrawPtr;
		ShaderProgram m_lineShader;
	};


	class IKNavigationSystem;
	class IKNavigationDebugDraw;
	class DebugDrawingSystem_ikNav : public DebugDrawingSystem {
	public:
		DebugDrawingSystem_ikNav() = default;
		void StartUp(IKNavigationSystem* ikNavSystemPtr)  noexcept override;
		void Draw(EventManager& eventManager, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) noexcept override;
		void ShutDown() noexcept override;
	private:
		std::unique_ptr<IKNavigationDebugDraw> m_ikNavDebugDrawPtr;
		IKNavigationSystem* m_ikNavSystemPtr;
		ShaderProgram m_lineShader;
	};
}

#endif

#endif