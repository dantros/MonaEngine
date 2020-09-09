#include "Renderer.hpp"
#include <imgui.h>
#include "examples/imgui_impl_glfw.h"
#include "examples/imgui_impl_opengl3.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/type_ptr.hpp>
#include "../Core/Log.hpp"
#include "ShaderProgram.hpp"
#include "ModelManager.hpp"
#include "../Core/RootDirectory.hpp"
void GLAPIENTRY MessageCallback(GLenum source,
								GLenum type,
								GLuint id,
								GLenum severity,
								GLsizei length,
								const GLchar* message,
								const void* userParam)
{
	MONA_LOG_ERROR("OpenGL Error: type = {0}, message = {1}",type,  message);
		
}


namespace Mona{
	template
		class ComponentManager<CameraComponent>;

	template
		class ComponentManager<TransformComponent>;
	template
		class ComponentManager<StaticMeshComponent>;



	void Renderer::StartUp(EventManager& eventManager) noexcept {
		ModelManager::GetInstance().StartUp();
		ShaderProgram shader(SourcePath("Assets/Shaders/BasicVS.vs"),
								SourcePath("Assets/Shaders/BasicPS.ps"));
		shader.UseProgram();
		m_onWindowResizeSubscription = eventManager.Subscribe(this, &Renderer::OnWindowResizeEvent);

		glEnable(GL_DEPTH_TEST);
		//glEnable(GL_CULL_FACE);
		StartDebugConfiguration();
		StartImGui();
	}
	void Renderer::ShutDown(EventManager& eventManager) noexcept {
		eventManager.Unsubscribe(m_onWindowResizeSubscription);
	}
	void Renderer::OnWindowResizeEvent(const WindowResizeEvent& event) {
		if (event.width == 0 || event.height == 0)
			return;
		glViewport(0, 0, event.width, event.height);
	}

	void Renderer::StartDebugConfiguration() noexcept {
		glEnable(GL_DEBUG_OUTPUT);
		glDebugMessageCallback(MessageCallback, 0);
		glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_NOTIFICATION, 0, nullptr, GL_FALSE);
	}

	void Renderer::StartImGui() noexcept {

		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImGuiIO& io = ImGui::GetIO(); (void)io;
		ImGui::StyleColorsDark();
		ImGui_ImplGlfw_InitForOpenGL(glfwGetCurrentContext(), true);
		ImGui_ImplOpenGL3_Init("#version 450");

	}

	void Renderer::RenderImGui(EventManager& eventManager) noexcept {
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		{
			ImGui::Begin("FPS Counter:");
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::End();
		}
		eventManager.Publish(DebugGUIEvent());
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	}

	void Renderer::Render(EventManager& eventManager,
				InnerComponentHandle cameraHandle,
				ComponentManager<StaticMeshComponent>& staticMeshDataManager,
				ComponentManager<TransformComponent>& transformDataManager,
				ComponentManager<CameraComponent>& cameraDataManager) noexcept 
	{
		
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glm::mat4 viewMatrix;
		glm::mat4 projectionMatrix;
		if (cameraDataManager.IsValid(cameraHandle)) {
			CameraComponent* camera = cameraDataManager.GetComponentPointer(cameraHandle);
			GameObject* cameraOwner = cameraDataManager.GetOwner(cameraHandle);
			TransformComponent* cameraTransform = transformDataManager.GetComponentPointer(cameraOwner->GetInnerComponentHandle<TransformComponent>());
			viewMatrix = cameraTransform->GetViewMatrixFromTransform();
			projectionMatrix = camera->GetProjectionMatrix();
		}
		else {
			MONA_LOG_INFO("Render Info: No camera has been set, using defaults transformations");
			viewMatrix = glm::mat4(1.0f);
			projectionMatrix = glm::perspective(glm::radians(50.0f), 16.0f / 9.0f, 0.1f, 100.0f);
		}
		glUniformMatrix4fv(3, 1, GL_FALSE, glm::value_ptr(viewMatrix));
		glUniformMatrix4fv(4, 1, GL_FALSE, glm::value_ptr(projectionMatrix));
		//Iterating over each static mesh
		for (decltype(staticMeshDataManager.GetCount()) i = 0;
			i < staticMeshDataManager.GetCount();
			i++)
		{
			StaticMeshComponent& staticMesh = staticMeshDataManager[i];
			GameObject* owner = staticMeshDataManager.GetOwnerByIndex(i);
			TransformComponent* transform = transformDataManager.GetComponentPointer(owner->GetInnerComponentHandle<TransformComponent>());
			auto& modelHandle = staticMesh.GetHandle();
			glUniformMatrix4fv(5, 1, GL_FALSE, glm::value_ptr(transform->GetModelMatrix()));
			glBindVertexArray(modelHandle.ID);
			glUniform3fv(2, 1, glm::value_ptr(staticMesh.GetColor()));
			glDrawElements(GL_TRIANGLES, modelHandle.count, GL_UNSIGNED_INT, 0);
			
		}
		
		
		RenderImGui(eventManager);
		
	}
}