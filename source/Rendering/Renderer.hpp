#pragma once
#ifndef RENDERER_HPP
#define RENDERER_HPP
#include <vector>
#include "../Event/EventManager.hpp"
#include "../World/ComponentTypes.hpp"
#include "../World/TransformComponent.hpp"
#include "../Animation/SkeletalMeshComponent.hpp"
#include "StaticMeshComponent.hpp"
#include "CameraComponent.hpp"
#include "ShaderProgram.hpp"
#include "DirectionalLightComponent.hpp"
#include "SpotLightComponent.hpp"
#include "PointLightComponent.hpp"
#include "Material.hpp"
#include "../DebugDrawing/DebugDrawingSystem.hpp"



namespace Mona {


	class Renderer {
	public:
		static constexpr int NUM_HALF_MAX_DIRECTIONAL_LIGHTS = 1;
		static constexpr int NUM_HALF_MAX_POINT_LIGHTS = 3;
		static constexpr int NUM_HALF_MAX_SPOT_LIGHTS = 3;
		Renderer() = default;
		void StartUp(EventManager& eventManager, DebugDrawingSystem* debugDrawingSystemPtr) noexcept;
		void Render(EventManager& eventManager,
					const InnerComponentHandle& cameraHandle,
					const glm::vec3& ambientLight,
					StaticMeshComponent::managerType& staticMeshDataManager,
					SkeletalMeshComponent::managerType& skeletalMeshDataManager,
					TransformComponent::managerType &transformDataManager,
					CameraComponent::managerType &cameraDataManager,
					DirectionalLightComponent::managerType &directionalLightDataManager,
					SpotLightComponent::managerType &spotLightDataManager,
					PointLightComponent::managerType &pointLightDataManager) noexcept;
		void ShutDown(EventManager& eventManager) noexcept;
		void OnWindowResizeEvent(const WindowResizeEvent& event);
		std::shared_ptr<Material> CreateMaterial(MaterialType type);
	private:
		struct DirectionalLight
		{
			glm::vec3 colorIntensity; //12
			float padding04; //16
			glm::vec3 direction; //28
			float padding08; //32
		};

		struct PointLight {
			glm::vec3 colorIntensity; //12
			float padding04; //16
			glm::vec3 position; //28
			float maxRadius; //32
		};

		struct SpotLight {
			glm::vec3 colorIntensity; //12
			float maxRadius; //16
			glm::vec3 position; //28
			float cosPenumbraAngle; //32
			glm::vec3 direction;
			float cosUmbraAngle; //48
		};

		struct Lights {
			SpotLight spotLights[2 * NUM_HALF_MAX_SPOT_LIGHTS];
			PointLight pointLights[2 * NUM_HALF_MAX_POINT_LIGHTS]; 
			DirectionalLight directionalLights[2 * NUM_HALF_MAX_DIRECTIONAL_LIGHTS]; 
			glm::vec3 ambientLight; 
			int spotLightsCount; 
			int pointLightsCount; 
			int directionalLightsCount; 
		};
		std::vector<ShaderProgram> m_shaders;
		SubscriptionHandle m_onWindowResizeSubscription;
		DebugDrawingSystem* m_debugDrawingSystemPtr = nullptr;
		unsigned int m_lightDataUBO = 0;

	};
}
#endif