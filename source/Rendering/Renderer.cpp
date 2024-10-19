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
#include "../Animation/SkinnedMesh.hpp"
#include "UnlitFlatMaterial.hpp"
#include "UnlitTexturedMaterial.hpp"
#include "DiffuseFlatMaterial.hpp"
#include "DiffuseTexturedMaterial.hpp"
#include "PBRFlatMaterial.hpp"
#include "PBRTexturedMaterial.hpp"

namespace Mona{
	template
		class ComponentManager<CameraComponent>;

	template
		class ComponentManager<TransformComponent>;
	template
		class ComponentManager<StaticMeshComponent>;



	void Renderer::StartUp(EventManager& eventManager, DebugDrawingSystem* debugDrawingSystemPtr) noexcept {

		constexpr unsigned int offset = static_cast<unsigned int>(MaterialType::MaterialTypeCount);
		//Construcción de todos los shaders que soporta el motor.
		m_shaders[static_cast<unsigned int>(MaterialType::UnlitFlat)]                = ShaderProgram(SourceDirectoryData::SourcePath("EngineAssets/Shaders/UnlitFlat.vs")               , SourceDirectoryData::SourcePath("EngineAssets/Shaders/UnlitFlat.ps"));
		m_shaders[static_cast<unsigned int>(MaterialType::UnlitTextured)]            = ShaderProgram(SourceDirectoryData::SourcePath("EngineAssets/Shaders/UnlitTextured.vs")           , SourceDirectoryData::SourcePath("EngineAssets/Shaders/UnlitTextured.ps"));
		m_shaders[static_cast<unsigned int>(MaterialType::DiffuseFlat)]              = ShaderProgram(SourceDirectoryData::SourcePath("EngineAssets/Shaders/DiffuseFlat.vs")             , SourceDirectoryData::SourcePath("EngineAssets/Shaders/DiffuseFlat.ps"));
		m_shaders[static_cast<unsigned int>(MaterialType::DiffuseTextured)]          = ShaderProgram(SourceDirectoryData::SourcePath("EngineAssets/Shaders/DiffuseTextured.vs")         , SourceDirectoryData::SourcePath("EngineAssets/Shaders/DiffuseTextured.ps"));
		m_shaders[static_cast<unsigned int>(MaterialType::PBRFlat)]                  = ShaderProgram(SourceDirectoryData::SourcePath("EngineAssets/Shaders/PBRFlat.vs")                 , SourceDirectoryData::SourcePath("EngineAssets/Shaders/PBRFlat.ps"));
		m_shaders[static_cast<unsigned int>(MaterialType::PBRTextured)]              = ShaderProgram(SourceDirectoryData::SourcePath("EngineAssets/Shaders/PBRTextured.vs")             , SourceDirectoryData::SourcePath("EngineAssets/Shaders/PBRTextured.ps"));
		m_shaders[static_cast<unsigned int>(MaterialType::UnlitFlat) + offset]       = ShaderProgram(SourceDirectoryData::SourcePath("EngineAssets/Shaders/UnlitFlatSkinning.vs")       , SourceDirectoryData::SourcePath("EngineAssets/Shaders/UnlitFlat.ps"));
		m_shaders[static_cast<unsigned int>(MaterialType::UnlitTextured) + offset]   = ShaderProgram(SourceDirectoryData::SourcePath("EngineAssets/Shaders/UnlitTexturedSkinning.vs")   , SourceDirectoryData::SourcePath("EngineAssets/Shaders/UnlitTextured.ps"));
		m_shaders[static_cast<unsigned int>(MaterialType::DiffuseFlat) + offset]     = ShaderProgram(SourceDirectoryData::SourcePath("EngineAssets/Shaders/DiffuseFlatSkinning.vs")     , SourceDirectoryData::SourcePath("EngineAssets/Shaders/DiffuseFlat.ps"));
		m_shaders[static_cast<unsigned int>(MaterialType::DiffuseTextured) + offset] = ShaderProgram(SourceDirectoryData::SourcePath("EngineAssets/Shaders/DiffuseTexturedSkinning.vs") , SourceDirectoryData::SourcePath("EngineAssets/Shaders/DiffuseTextured.ps"));
		m_shaders[static_cast<unsigned int>(MaterialType::PBRFlat) + offset]         = ShaderProgram(SourceDirectoryData::SourcePath("EngineAssets/Shaders/PBRFlatSkinning.vs")         , SourceDirectoryData::SourcePath("EngineAssets/Shaders/PBRFlat.ps"));
		m_shaders[static_cast<unsigned int>(MaterialType::PBRTextured) + offset]     = ShaderProgram(SourceDirectoryData::SourcePath("EngineAssets/Shaders/PBRTexturedSkinning.vs")     , SourceDirectoryData::SourcePath("EngineAssets/Shaders/PBRTextured.ps"));

		//El sistema de rendering debe subscribirse al cambio de resoluci�n de la ventana para actulizar la resoluci�n
		//del framebuffer al que OpenGL renderiza.
		eventManager.Subscribe(m_onWindowResizeSubscription, this, &Renderer::OnWindowResizeEvent);
		m_debugDrawingSystemPtr = debugDrawingSystemPtr;
		m_currentMatrixPalette.resize(NUM_MAX_BONES, glm::mat4(1.0f));
		glEnable(GL_DEPTH_TEST);

		//Se genera el buffer que contendra toda la información lumínica de la escena
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
		const glm::vec3& ambientLight,
		ComponentManager<StaticMeshComponent>& staticMeshDataManager,
		ComponentManager<SkeletalMeshComponent>& skeletalMeshDataManager,
		ComponentManager<TransformComponent>& transformDataManager,
		ComponentManager<CameraComponent>& cameraDataManager,
		ComponentManager<DirectionalLightComponent>& directionalLightDataManager,
		ComponentManager<SpotLightComponent>& spotLightDataManager,
		ComponentManager<PointLightComponent>& pointLightDataManager) noexcept
	{
		glClearColor(m_backgroundColor[0], m_backgroundColor[1], m_backgroundColor[2], m_backgroundColor[3]);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glm::mat4 viewMatrix;
		glm::mat4 projectionMatrix;
		glm::vec3 cameraPosition = glm::vec3(0.0f);
		if (cameraDataManager.IsValid(cameraHandle)) {
			//Si el usuario configuro la camara principal configuramos apartir de esta la matriz de vista y projecci�n
			//viewMatrix y projectionMatrix respectivamente
			const CameraComponent* camera = cameraDataManager.GetComponentPointer(cameraHandle);
			GameObject* cameraOwner = cameraDataManager.GetOwner(cameraHandle);
			TransformComponent* cameraTransform = transformDataManager.GetComponentPointer(cameraOwner->GetInnerComponentHandle<TransformComponent>());
			viewMatrix = cameraTransform->GetViewMatrixFromTransform();
			projectionMatrix = camera->GetProjectionMatrix();
			cameraPosition = cameraTransform->GetLocalTranslation();
		}
		else {
			//En caso de que el usuario no haya configurado una cama principal usamos valores predeterminados para ambas matrices
			MONA_LOG_INFO("Render Info: No camera has been set, using defaults transformations");
			viewMatrix = glm::mat4(1.0f);
			projectionMatrix = glm::perspective(glm::radians(50.0f), 16.0f / 9.0f, 0.1f, 100.0f);
		}



		//Comienza carga en CPU de la informaci�n lum�nica de la escena
		Lights lights;
		lights.ambientLight = ambientLight;

		//Se pasa la informacion de a lo mas las primeras NUM_HALF_MAX_DIRECTIONAL_LIGHTS * 2 componentes de luz direccional
		//A una instancia de Lights (informacion de la escena en CPU)
		uint32_t directionalLightsCount = std::min(static_cast<uint32_t>(NUM_HALF_MAX_DIRECTIONAL_LIGHTS * 2), directionalLightDataManager.GetCount());
		lights.directionalLightsCount = static_cast<int>(directionalLightsCount);
		for (uint32_t i = 0; i < directionalLightsCount; i++) {
			const DirectionalLightComponent& dirLight = directionalLightDataManager[i];
			GameObject* dirLightOwner = directionalLightDataManager.GetOwnerByIndex(i);
			TransformComponent* lightTransform = transformDataManager.GetComponentPointer(dirLightOwner->GetInnerComponentHandle<TransformComponent>());
			lights.directionalLights[i].colorIntensity = dirLight.GetLightColor();
			lights.directionalLights[i].direction = glm::rotate(dirLight.GetLightDirection(), lightTransform->GetFrontVector());
		}

		//Lo mismo para spotlights
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

		//Finalmente luces puntuales
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

		//Pasamos la informacion lum�nica a GPU con un unico llamado a OpenGL fuera de los loops de las primitivas.
		glBindBuffer(GL_UNIFORM_BUFFER, m_lightDataUBO);
		glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(Lights), &lights);
		glBindBuffer(GL_UNIFORM_BUFFER, 0);
		//Iteraci�n sobre todas las instancias de StaticMeshComponent
		for (decltype(staticMeshDataManager.GetCount()) i = 0;
			i < staticMeshDataManager.GetCount();
			i++)
		{
			StaticMeshComponent& staticMesh = staticMeshDataManager[i];
			GameObject* owner = staticMeshDataManager.GetOwnerByIndex(i);
			//Se obtiene la informaci�n espacial para configurar la matriz de modelo dentro del shader.
			TransformComponent* transform = transformDataManager.GetComponentPointer(owner->GetInnerComponentHandle<TransformComponent>());
			//Configuraci�n de la malla a ser renderizada y las uniformes asociadas a su material.
			glBindVertexArray(staticMesh.GetMeshVAOID());
			staticMesh.m_materialPtr->SetUniforms(projectionMatrix, viewMatrix, transform->GetModelMatrix(), cameraPosition);
			glDrawElements(GL_TRIANGLES, staticMesh.GetMeshIndexCount(), GL_UNSIGNED_INT, 0);
			
		}
		
		//Iteracion sobre todas las instancias de SkeletalMeshComponent
		for (decltype(skeletalMeshDataManager.GetCount()) i = 0;
			i < skeletalMeshDataManager.GetCount();
			i++)
		{
			SkeletalMeshComponent& skeletalMesh = skeletalMeshDataManager[i];
			GameObject* owner = skeletalMeshDataManager.GetOwnerByIndex(i);
			TransformComponent* transform = transformDataManager.GetComponentPointer(owner->GetInnerComponentHandle<TransformComponent>());
			auto skinnedMesh = skeletalMesh.m_skinnedMeshPtr;
			glBindVertexArray(skinnedMesh->GetVertexArrayID());
			//A diferencias de StaticMeshes, SkeletalMeshComponent necesita configurar las paletas de matrices de animacion
			//estas se le solicitan al animationController
			auto &animController = skeletalMesh.GetAnimationController();
			skeletalMesh.m_materialPtr->SetUniforms(projectionMatrix, viewMatrix, transform->GetModelMatrix(), cameraPosition);
			animController.GetMatrixPalette(m_currentMatrixPalette);
			glUniformMatrix4fv(ShaderProgram::BoneTransformShaderLocation, skeletalMesh.GetSkeleton()->JointCount(), GL_FALSE, (GLfloat*) m_currentMatrixPalette.data());
			glDrawElements(GL_TRIANGLES, skinnedMesh->GetIndexBufferCount(), GL_UNSIGNED_INT, 0);
		}
		//En no Debub build este llamado es vacio, en caso contrario se renderiza informaci�n de debug
		m_debugDrawingSystemPtr->Draw(eventManager, viewMatrix, projectionMatrix);
		
	}

	std::shared_ptr<Material> Renderer::CreateMaterial(MaterialType type, bool isForSkinning) {

		unsigned int offset = static_cast<unsigned int>(type);
		offset = isForSkinning ? offset + static_cast<unsigned int>(MaterialType::MaterialTypeCount) : offset;
		switch (type)
		{
		case Mona::MaterialType::UnlitFlat:
			return std::make_shared<UnlitFlatMaterial>(m_shaders[offset], isForSkinning);
			break;
		case Mona::MaterialType::UnlitTextured:
			return std::make_shared<UnlitTexturedMaterial>(m_shaders[offset], isForSkinning);
			break;
		case Mona::MaterialType::DiffuseFlat:
			return std::make_shared<DiffuseFlatMaterial>(m_shaders[offset], isForSkinning);
			break;
		case Mona::MaterialType::DiffuseTextured:
			return std::make_shared<DiffuseTexturedMaterial>(m_shaders[offset], isForSkinning);
			break;
		case Mona::MaterialType::PBRFlat:
			return std::make_shared<PBRFlatMaterial>(m_shaders[offset], isForSkinning);
			break;
		case Mona::MaterialType::PBRTextured:
			return std::make_shared<PBRTexturedMaterial>(m_shaders[offset], isForSkinning);
			break;
		case Mona::MaterialType::MaterialTypeCount:
			return std::make_shared<DiffuseFlatMaterial>(m_shaders[offset], isForSkinning);
			break;
		default:
			return nullptr;
			break;
		}
	}

	void Renderer::SetBackgroundColor(float r, float g, float b, float alpha) {
		m_backgroundColor = glm::vec4(r, g, b, alpha);
	}
}