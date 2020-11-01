#include "Renderer.hpp"
#include <imgui.h>
#include "examples/imgui_impl_glfw.h"
#include "examples/imgui_impl_opengl3.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/type_ptr.hpp>
#include "../Core/Log.hpp"
#include "Mesh.hpp"
#include "../Core/RootDirectory.hpp"

namespace Mona{
	template
		class ComponentManager<CameraComponent>;

	template
		class ComponentManager<TransformComponent>;
	template
		class ComponentManager<StaticMeshComponent>;



	void Renderer::StartUp(EventManager& eventManager, DebugDrawingSystem* debugDrawingSystemPtr) noexcept {
		m_shader = ShaderProgram(SourcePath("Assets/Shaders/BasicVS.vs"),
			SourcePath("Assets/Shaders/BasicPS.ps"));
		m_onWindowResizeSubscription = eventManager.Subscribe(this, &Renderer::OnWindowResizeEvent);
		m_debugDrawingSystemPtr = debugDrawingSystemPtr;
		glEnable(GL_DEPTH_TEST);
		//glEnable(GL_CULL_FACE);
	}
	void Renderer::ShutDown(EventManager& eventManager) noexcept {
		eventManager.Unsubscribe(m_onWindowResizeSubscription);
	}
	void Renderer::OnWindowResizeEvent(const WindowResizeEvent& event) {
		if (event.width == 0 || event.height == 0)
			return;
		glViewport(0, 0, event.width, event.height);
	}

	void Renderer::Render(EventManager& eventManager,
				const InnerComponentHandle& cameraHandle,
				ComponentManager<StaticMeshComponent>& staticMeshDataManager,
				ComponentManager<TransformComponent>& transformDataManager,
				ComponentManager<CameraComponent>& cameraDataManager) noexcept 
	{
		
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		m_shader.UseProgram();
		glm::mat4 viewMatrix;
		glm::mat4 projectionMatrix;
		if (cameraDataManager.IsValid(cameraHandle)) {
			const CameraComponent* camera = cameraDataManager.GetComponentPointer(cameraHandle);
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
			glUniformMatrix4fv(5, 1, GL_FALSE, glm::value_ptr(transform->GetModelMatrix()));
			glBindVertexArray(staticMesh.GetMeshVAOID());
			glUniform3fv(2, 1, glm::value_ptr(staticMesh.GetColor()));
			glDrawElements(GL_TRIANGLES, staticMesh.GetMeshIndexCount(), GL_UNSIGNED_INT, 0);
			
		}
		
		m_debugDrawingSystemPtr->Draw(eventManager, viewMatrix, projectionMatrix);
		
	}
}