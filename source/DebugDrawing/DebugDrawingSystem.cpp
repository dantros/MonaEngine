#include "DebugDrawingSystem.hpp"



#ifndef NDEBUG
#include "../Core/Log.hpp"
#include <imgui.h>
#include "examples/imgui_impl_glfw.h"
#include "examples/imgui_impl_opengl3.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/type_ptr.hpp>
#include "../PhysicsCollision/PhysicsCollisionSystem.hpp"
#include "../Core/RootDirectory.hpp"
void GLAPIENTRY MessageCallback(GLenum source,
	GLenum type,
	GLuint id,
	GLenum severity,
	GLsizei length,
	const GLchar* message,
	const void* userParam)
{
	MONA_LOG_ERROR("OpenGL Error: type = {0}, message = {1}", type, message);

}

namespace Mona {


	void DebugDrawingSystem::Draw(EventManager& eventManager, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) noexcept {

		glUseProgram(m_lineShader.GetProgramID());
		glUniformMatrix4fv(0, 1, GL_FALSE, glm::value_ptr(projectionMatrix));
		glUniformMatrix4fv(1, 1, GL_FALSE, glm::value_ptr(viewMatrix));
		m_physicsWorldPtr->debugDrawWorld();



		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		{
			ImGui::Begin("Debug Settings:");
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::Separator();
			ImGui::Text("Collision Physics System Debug Draw (Bullet):");
			ImGui::Checkbox("Draw Wireframe", &(m_bulletDebugDrawPtr->m_bDrawWireframe));
			ImGui::Checkbox("Draw ContactPoints", &(m_bulletDebugDrawPtr->m_bDrawContactsPoints));
			ImGui::Checkbox("Draw AABB", &(m_bulletDebugDrawPtr->m_bDrawAABB));
			ImGui::End();
		}
		eventManager.Publish(DebugGUIEvent());
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	}
	void DebugDrawingSystem::StartUp(PhysicsCollisionSystem* physicsSystemPtr) noexcept {
		m_lineShader = ShaderProgram(SourcePath("source/Rendering/Shaders/LineVS.vs"),
			SourcePath("source/Rendering/Shaders/LinePS.ps"));
		m_physicsWorldPtr = physicsSystemPtr->GetPhysicsWorldPtr();
		m_bulletDebugDrawPtr.reset(new BulletDebugDraw());
		m_bulletDebugDrawPtr->StartUp();
		m_bulletDebugDrawPtr->setDebugMode(0);
		m_physicsWorldPtr->setDebugDrawer(m_bulletDebugDrawPtr.get());

		glEnable(GL_DEBUG_OUTPUT);
		glDebugMessageCallback(MessageCallback, 0);
		glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_NOTIFICATION, 0, nullptr, GL_FALSE);

		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImGuiIO& io = ImGui::GetIO(); (void)io;
		ImGui::StyleColorsDark();
		ImGui_ImplGlfw_InitForOpenGL(glfwGetCurrentContext(), true);
		ImGui_ImplOpenGL3_Init("#version 450");
	}

	void DebugDrawingSystem::ShutDown() noexcept {
		ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
	}
}

#endif
