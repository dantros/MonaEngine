#include "Renderer.hpp"
#include <imgui.h>
#include "examples/imgui_impl_glfw.h"
#include "examples/imgui_impl_opengl3.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
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


		glGenBuffers(1, &m_lightDataUBO);
		glBindBuffer(GL_UNIFORM_BUFFER, m_lightDataUBO);
		glBufferData(GL_UNIFORM_BUFFER, sizeof(Lights), NULL, GL_DYNAMIC_DRAW);
		glBindBuffer(GL_UNIFORM_BUFFER, 0);
		glBindBufferBase(GL_UNIFORM_BUFFER, 0, m_lightDataUBO);
	}
	void Renderer::ShutDown(EventManager& eventManager) noexcept {
		eventManager.Unsubscribe(m_onWindowResizeSubscription);
		glDeleteBuffers(1, &m_lightDataUBO);
	}
	void Renderer::OnWindowResizeEvent(const WindowResizeEvent& event) {
		if (event.width == 0 || event.height == 0)
			return;
		glViewport(0, 0, event.width, event.height);
	}

	void Renderer::Render(EventManager& eventManager,
		const InnerComponentHandle& cameraHandle,
		const glm::vec3& ambientLightColorIntensity,
		StaticMeshComponent::managerType& staticMeshDataManager,
		TransformComponent::managerType& transformDataManager,
		CameraComponent::managerType& cameraDataManager,
		DirectionalLightComponent::managerType& directionalLightDataManager,
		SpotLightComponent::managerType& spotLightDataManager,
		PointLightComponent::managerType& pointLightDataManager) noexcept
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glm::mat4 viewMatrix;
		glm::mat4 projectionMatrix;
		glm::vec3 spotLightPosition = glm::vec3(0.f);
		glm::vec3 spotLightDirection = glm::vec3(0.0f, -1.0f, 0.0f);
		if (cameraDataManager.IsValid(cameraHandle)) {
			//Si el usuario configuro la camara principal configuramos apartir de esta la matrix de vista y projección
			//viewMatrix y projectionMatrix respectivamente
			const CameraComponent* camera = cameraDataManager.GetComponentPointer(cameraHandle);
			GameObject* cameraOwner = cameraDataManager.GetOwner(cameraHandle);
			TransformComponent* cameraTransform = transformDataManager.GetComponentPointer(cameraOwner->GetInnerComponentHandle<TransformComponent>());
			viewMatrix = cameraTransform->GetViewMatrixFromTransform();
			projectionMatrix = camera->GetProjectionMatrix();
			spotLightPosition = cameraTransform->GetLocalTranslation();
			spotLightDirection = cameraTransform->GetFrontVector();
		}
		else {
			//En caso de que el usuario no haya configurado una cama principal usamos valores predeterminador para ambas matrices
			MONA_LOG_INFO("Render Info: No camera has been set, using defaults transformations");
			viewMatrix = glm::mat4(1.0f);
			projectionMatrix = glm::perspective(glm::radians(50.0f), 16.0f / 9.0f, 0.1f, 100.0f);
		}




		Lights lights;
		lights.ambientLightColorIntensity = ambientLightColorIntensity;
		uint32_t directionalLightsCount = std::min(static_cast<uint32_t>(NUM_HALF_MAX_DIRECTIONAL_LIGHTS * 2), directionalLightDataManager.GetCount());
		lights.directionalLightsCount = static_cast<int>(directionalLightsCount);
		for (uint32_t i = 0; i < directionalLightsCount; i++) {
			const DirectionalLightComponent& dirLight = directionalLightDataManager[i];
			GameObject* dirLightOwner = directionalLightDataManager.GetOwnerByIndex(i);
			TransformComponent* lightTransform = transformDataManager.GetComponentPointer(dirLightOwner->GetInnerComponentHandle<TransformComponent>());
			lights.directionalLights[i].colorIntensity = dirLight.GetLightColor();
			lights.directionalLights[i].direction = glm::rotate(dirLight.GetLightDirection(), lightTransform->GetFrontVector());
		}

		uint32_t spotLightsCount = std::min(static_cast<uint32_t>(NUM_HALF_MAX_SPOT_LIGHTS * 2), spotLightDataManager.GetCount());
		lights.spotLightsCount = static_cast<int>(spotLightsCount);
		for (uint32_t i = 0; i < spotLightsCount; i++) {
			const SpotLightComponent& spotLight = spotLightDataManager[i];
			GameObject* spotLightOwner = spotLightDataManager.GetOwnerByIndex(i);
			TransformComponent* lightTransform = transformDataManager.GetComponentPointer(spotLightOwner->GetInnerComponentHandle<TransformComponent>());
			lights.spotLights[i].colorIntensity = spotLight.GetLightColor();
			lights.spotLights[i].direction = glm::rotate(spotLight.GetLightDirection(), lightTransform->GetFrontVector());
			lights.spotLights[i].position = lightTransform->GetLocalTranslation();
			lights.spotLights[i].cosPenumbraAngle = glm::cos(spotLight.GetPenumbraAngle());
			lights.spotLights[i].cosUmbraAngle = glm::cos(spotLight.GetUmbraAngle());
			lights.spotLights[i].maxRadius = spotLight.GetMaxRadius();
		}

		uint32_t pointLightsCount = std::min(static_cast<uint32_t>(NUM_HALF_MAX_POINT_LIGHTS * 2), pointLightDataManager.GetCount());
		lights.pointLightsCount = static_cast<int>(pointLightsCount);
		for (uint32_t i = 0; i < pointLightsCount; i++) {
			const PointLightComponent& pointLight = pointLightDataManager[i];
			GameObject* pointLightOwner = pointLightDataManager.GetOwnerByIndex(i);
			TransformComponent* lightTransform = transformDataManager.GetComponentPointer(pointLightOwner->GetInnerComponentHandle<TransformComponent>());
			lights.pointLights[i].colorIntensity = pointLight.GetLightColor();
			lights.pointLights[i].position = lightTransform->GetLocalTranslation();
			lights.pointLights[i].maxRadius = pointLight.GetMaxRadius();
		}
		/*
		
		lights.directionalLights[0].colorIntensity = glm::vec3(0.0f);
		lights.directionalLights[0].direction = glm::normalize(glm::vec3(-1.0f, 1.0f, -2.0f));
		lights.directionalLights[1].colorIntensity = glm::vec3(0.2f);
		lights.directionalLights[1].direction = glm::normalize(glm::vec3(-1.0f, -1.0f, -2.0f));
		lights.directionalLightsCount = 2;
		lights.pointLights[0].colorIntensity = glm::vec3(0.0f);
		lights.pointLights[0].position = glm::vec3(0.0f,0.0f, -7.0f);
		lights.pointLights[0].maxRadius = 5.0f;
		lights.pointLights[1].colorIntensity = glm::vec3(0.0f);
		lights.pointLights[1].position = glm::vec3(-3.0f, -3.0f, 7.0f);
		lights.pointLights[1].maxRadius = 200.0f;
		lights.pointLightsCount = 2;
		lights.spotLights[0].colorIntensity = glm::vec3(10.f);
		lights.spotLights[0].direction = spotLightDirection;
		lights.spotLights[0].position = spotLightPosition;
		lights.spotLights[0].cosPenumbraAngle = glm::cos(glm::radians(20.0f));
		lights.spotLights[0].cosUmbraAngle = glm::cos(glm::radians(25.0f));
		lights.spotLights[0].maxRadius = 10.0f;
		lights.spotLightsCount = 1;*/

		glBindBuffer(GL_UNIFORM_BUFFER, m_lightDataUBO);
		glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(Lights), &lights);
		glBindBuffer(GL_UNIFORM_BUFFER, 0);
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