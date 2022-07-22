#include "MonaEngine.hpp"
#include "Utilities/BasicCameraControllers.hpp"
#include "Rendering/UnlitFlatMaterial.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"
#include "Rendering/DiffuseTexturedMaterial.hpp"
#include "Rendering/PBRTexturedMaterial.hpp"
#include <numbers>
#include <imgui.h>


float gaussian(float x, float y, float s, float sigma, glm::vec2 mu) {
	return (s / (sigma * std::sqrt(2 * std::numbers::pi))) * std::exp((-1 / (2 * std::pow(sigma, 2))) * (std::pow((x - mu[0]), 2) + std::pow((y - mu[1]), 2)));
}

void AddDirectionalLight(Mona::World& world, const glm::vec3& axis, float angle, float lightIntensity)
{
	auto light = world.CreateGameObject<Mona::GameObject>();
	auto transform = world.AddComponent<Mona::TransformComponent>(light);
	transform->Rotate(axis, angle);
	//transform->SetTranslation(center);
	world.AddComponent<Mona::DirectionalLightComponent>(light, lightIntensity * glm::vec3(1.0f));

}

Mona::GameObjectHandle<Mona::GameObject> AddTerrain(Mona::World& world) {
	auto terrain = world.CreateGameObject<Mona::GameObject>();
	auto& meshManager = Mona::MeshManager::GetInstance();
	auto materialPtr = std::static_pointer_cast<Mona::UnlitFlatMaterial>(world.CreateMaterial(Mona::MaterialType::UnlitFlat));
	materialPtr->SetColor(glm::vec3(0.3, 0.5f, 0.7f));
	//float planeScale = 10.0f;
	auto transform = world.AddComponent<Mona::TransformComponent>(terrain);
	//transform->SetScale(glm::vec3(planeScale));
	glm::vec2 minXY(-10, -10);
	glm::vec2 maxXY(10, 10);
	int numInnerVerticesWidth = 500;
	int numInnerVerticesHeight = 500;
	auto heighFunc = [](float x, float y) -> float {
		return gaussian(x, y, 5, 10, { 2, 3 });
	};

	world.AddComponent<Mona::StaticMeshComponent>(terrain, meshManager.GenerateTerrain(minXY, maxXY, numInnerVerticesWidth,
		numInnerVerticesHeight, heighFunc, true, false), materialPtr);
	return terrain;
}

class IKRigCharacter : public Mona::GameObject
{
private:

	void UpdateMovement(Mona::World& world) {
		auto& input = world.GetInput();
		if (input.IsMouseButtonPressed(MONA_MOUSE_BUTTON_1) && !m_prevIsPress) {
			auto mousePos = input.GetMousePosition();

		}
		else if (m_prevIsPress && !input.IsMouseButtonPressed(MONA_MOUSE_BUTTON_1)) {
			m_prevIsPress = false;
		}

	}

	void UpdateAnimationState() {
		auto& animController = m_skeletalMesh->GetAnimationController();


	}
public:
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept {
		UpdateAnimationState();
	};
	virtual void UserStartUp(Mona::World& world) noexcept {
		auto& eventManager = world.GetEventManager();
		eventManager.Subscribe(m_debugGUISubcription, this, &IKRigCharacter::OnDebugGUIEvent);

		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		
		m_targetPosition = glm::vec3(0.0f);
		glm::fquat offsetRotation = glm::rotate(glm::fquat(1.0f, 0.0f, 0.0f, 0.0f), glm::radians(180.f), glm::vec3(0.0f, 0.0f, 1.0f)) * glm::rotate(glm::fquat(1.0f, 0.0f, 0.0f, 0.0f), glm::radians(-90.f), glm::vec3(1.0f, 0.0f, 0.0f));

		world.SetAudioListenerTransform(m_transform, offsetRotation);

		auto materialPtr = std::static_pointer_cast<Mona::DiffuseTexturedMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseTextured, true));
		auto& textureManager = Mona::TextureManager::GetInstance();
		auto diffuseTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Models/akai/akai_diffuse.png"));
		materialPtr->SetMaterialTint(glm::vec3(0.1f));
		materialPtr->SetDiffuseTexture(diffuseTexture);

		auto& meshManager = Mona::MeshManager::GetInstance();
		auto& skeletonManager = Mona::SkeletonManager::GetInstance();
		auto& animationManager = Mona::AnimationClipManager::GetInstance();
		auto skeleton = skeletonManager.LoadSkeleton(Mona::SourcePath("Assets/Models/akai_e_espiritu.fbx"));
		auto skinnedMesh = meshManager.LoadSkinnedMesh(skeleton, Mona::SourcePath("Assets/Models/akai_e_espiritu.fbx"), true);
		m_walkingAnimation = animationManager.LoadAnimationClip(Mona::SourcePath("Assets/Animations/female/walking.fbx"), skeleton);
		m_skeletalMesh = world.AddComponent<Mona::SkeletalMeshComponent>(*this, skinnedMesh, m_walkingAnimation, materialPtr);
		Mona::RigData rigData;
		rigData.scale = 3.0f;
		rigData.leftLeg.baseJointName = "Hips";
		rigData.leftLeg.endEffectorName = "LeftFoot";
		rigData.rightLeg.baseJointName = "Hips";
		rigData.rightLeg.endEffectorName = "RighFoot";
		rigData.hipJointName = "Hips";
		m_ikNavHandle = world.AddComponent<Mona::IKNavigationComponent>(*this, rigData, m_walkingAnimation);

	}

	void OnDebugGUIEvent(const Mona::DebugGUIEvent& event) {
		ImGui::Begin("Character Options:");
		ImGui::SliderFloat("DeacelerationFactor", &(m_deacelerationFactor), 0.0f, 10.0f);
		ImGui::SliderFloat("DistanceThreshold", &(m_distanceThreshold), 0.0f, 10.0f);
		ImGui::SliderFloat("AngularVelocityFactor", &(m_angularVelocityFactor), 0.0f, 10.0f);
		ImGui::SliderFloat("FadeTime", &(m_fadeTime), 0.0f, 1.0f);
		ImGui::End();
	}
private:
	float m_deacelerationFactor = 1.1f;
	float m_angularVelocityFactor = 3.0f;
	float m_distanceThreshold = 0.65f;
	float m_fadeTime = 0.5f;
	bool m_prevIsPress = false;
	glm::vec3 m_targetPosition = glm::vec3(0.0f);
	glm::vec3 m_targetFrontVector = glm::vec3(0.0f, -1.0f, 0.0f);
	Mona::TransformHandle m_transform;
	Mona::SkeletalMeshHandle m_skeletalMesh;
	Mona::IKNavigationHandle m_ikNavHandle;
	std::shared_ptr<Mona::AnimationClip> m_walkingAnimation;
	Mona::SubscriptionHandle m_debugGUISubcription;

};

class IKNav : public Mona::Application
{
public:
	IKNav() = default;
	~IKNav() = default;
	Mona::GameObjectHandle<Mona::BasicPerspectiveCamera> m_camera;
	virtual void UserStartUp(Mona::World &world) noexcept override{
		m_camera = world.CreateGameObject<Mona::BasicPerspectiveCamera>();
		world.SetAmbientLight(glm::vec3(0.2f));
		//world.AddComponent<Mona::SpotLightComponent>(m_camera, glm::vec3(100.0f), 5.0f, glm::radians(25.0f), glm::radians(37.0f));
		world.SetMainCamera(world.GetComponentHandle<Mona::CameraComponent>(m_camera));
		//AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(-180.0f), glm::vec3(0.0f, 0.0f, 1000.0f),5.0f);
		//AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(180.0f), glm::vec3(0.0f, 0.0f, 1000.0f),5.0f);
		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(-45.0f), 2);
		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(-135.0f), 2);
		//AddDirectionalLight(world, glm::vec3(0.0f, 1.0f, 0.0f), glm::radians(-135.0f), 15.0f);
		auto character = world.CreateGameObject<IKRigCharacter>();
		auto terrainObject = AddTerrain(world);
		world.GetComponentHandle<Mona::IKNavigationComponent>(character)->AddTerrain(terrainObject);

		
	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
	}
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept override {
		auto& input = world.GetInput();
		auto& window = world.GetWindow();
		if (input.IsKeyPressed(MONA_KEY_G))
		{
			window.SetFullScreen(true);
		}
		else if (input.IsKeyPressed(MONA_KEY_H))
		{
			window.SetFullScreen(false);
		}
		else if (input.IsKeyPressed(MONA_KEY_J))
		{
			window.SetWindowDimensions(glm::ivec2(1000, 1000));
		}
		else if (input.IsKeyPressed(MONA_KEY_1)) {
			m_camera->SetActive(false);
			input.SetCursorType(Mona::Input::CursorType::Normal);
		}
		else if (input.IsKeyPressed(MONA_KEY_2)) {
			m_camera->SetActive(true);
			input.SetCursorType(Mona::Input::CursorType::Disabled);
		}
	}
};
int main()
{	
	IKNav app;
	Mona::Engine engine(app);
	engine.StartMainLoop();
	
}