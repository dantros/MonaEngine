#include "Renderer.hpp"
#include <imgui.h>
#include "examples/imgui_impl_glfw.h"
#include "examples/imgui_impl_opengl3.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/type_ptr.hpp>
#include "../Core/Log.hpp"
#include "../Core/RootDirectory.hpp"
#include "../DebugDrawing/DebugDrawingSystem.hpp"
#include "Mesh.hpp"
#include "FlatColorMaterial.hpp"
#include "TextureMaterial.hpp"

namespace Mona{
	template
		class ComponentManager<CameraComponent>;

	template
		class ComponentManager<TransformComponent>;
	template
		class ComponentManager<StaticMeshComponent>;



	void Renderer::StartUp(EventManager& eventManager, DebugDrawingSystem* debugDrawingSystemPtr) noexcept {
	
		//Construcción de todos los shaders que soporta el motor.
		m_shaders.emplace_back(SourcePath("Assets/Shaders/BasicVS.vs"), SourcePath("Assets/Shaders/BasicPS.ps"));
		m_shaders.emplace_back(SourcePath("Assets/Shaders/TexturedModel.vs"), SourcePath("Assets/Shaders/TexturedModel.ps"));
		
		//El sistema de rendering debe subscribirse al cambio de resolución de la ventana para actulizar la resolución
		//del framebuffer al que OpenGL renderiza.
		m_onWindowResizeSubscription = eventManager.Subscribe(this, &Renderer::OnWindowResizeEvent);
		m_debugDrawingSystemPtr = debugDrawingSystemPtr;
		glEnable(GL_DEPTH_TEST);
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
		glm::mat4 viewMatrix;
		glm::mat4 projectionMatrix;
		if (cameraDataManager.IsValid(cameraHandle)) {
			//Si el usuario configuro la camara principal configuramos apartir de esta la matrix de vista y projección
			//viewMatrix y projectionMatrix respectivamente
			const CameraComponent* camera = cameraDataManager.GetComponentPointer(cameraHandle);
			GameObject* cameraOwner = cameraDataManager.GetOwner(cameraHandle);
			TransformComponent* cameraTransform = transformDataManager.GetComponentPointer(cameraOwner->GetInnerComponentHandle<TransformComponent>());
			viewMatrix = cameraTransform->GetViewMatrixFromTransform();
			projectionMatrix = camera->GetProjectionMatrix();
		}
		else {
			//En caso de que el usuario no haya configurado una cama principal usamos valores predeterminador para ambas matrices
			MONA_LOG_INFO("Render Info: No camera has been set, using defaults transformations");
			viewMatrix = glm::mat4(1.0f);
			projectionMatrix = glm::perspective(glm::radians(50.0f), 16.0f / 9.0f, 0.1f, 100.0f);
		}

		//Iteración sobre todas las instancias de StaticMeshComponent
		for (decltype(staticMeshDataManager.GetCount()) i = 0;
			i < staticMeshDataManager.GetCount();
			i++)
		{
			StaticMeshComponent& staticMesh = staticMeshDataManager[i];
			GameObject* owner = staticMeshDataManager.GetOwnerByIndex(i);
			//Se obtiene la información espacial para configurar la matriz de modelo dentro del shader.
			TransformComponent* transform = transformDataManager.GetComponentPointer(owner->GetInnerComponentHandle<TransformComponent>());
			//Configuración de la malla a ser renderizada y las uniformes asociadas a su material.
			glBindVertexArray(staticMesh.GetMeshVAOID());
			staticMesh.m_materialPtr->SetUniforms(projectionMatrix, viewMatrix, transform->GetModelMatrix());
			glDrawElements(GL_TRIANGLES, staticMesh.GetMeshIndexCount(), GL_UNSIGNED_INT, 0);
			
		}
		
		//En no Debub build este llamado es vacio, en caso contrario se renderiza información de debug
		m_debugDrawingSystemPtr->Draw(eventManager, viewMatrix, projectionMatrix);
		
	}

	std::shared_ptr<Material> Renderer::CreateMaterial(MaterialType type) {

		switch (type)
		{
		case Mona::MaterialType::FlatColor:
			return std::make_shared<FlatColorMaterial>(m_shaders[0].GetProgramID());
			break;
		case Mona::MaterialType::Textured:
			return std::make_shared<TextureMaterial>(m_shaders[1].GetProgramID());
			break;
		default:
			return nullptr;
			break;
		}
	}
}