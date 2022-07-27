#include "MonaEngine.hpp"
#include "Utilities/BasicCameraControllers.hpp"
#include "Rendering/UnlitFlatMaterial.hpp"
#include "Rendering/DiffuseFlatMaterial.hpp"
#include "Rendering/DiffuseTexturedMaterial.hpp"
#include "Rendering/PBRTexturedMaterial.hpp"
#include <numbers>
#include <imgui.h>
#include <random>


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
	auto materialPtr = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
	materialPtr->SetDiffuseColor(glm::vec3(0.03, 0.1f, 0.17f));
	//float planeScale = 10.0f;
	auto transform = world.AddComponent<Mona::TransformComponent>(terrain);
	//transform->SetScale(glm::vec3(planeScale));
	glm::vec2 minXY(-100, -100);
	glm::vec2 maxXY(100, 100);
	int numInnerVerticesWidth = 100;
	int numInnerVerticesHeight = 100;
	auto heighFunc = [](float x, float y) -> float {
		float result = 0;
		int funcNum = 100;
		glm::vec2 minXY(-100, -100);
		glm::vec2 maxXY(100, 100);
		float minHeight = -15;
		float maxHeight = 70;
		float minSigma = 3;
		float maxSigma = 20;
		std::srand(5);
		for (int i = 0; i < funcNum; i++) {
			float randMax = RAND_MAX;
			result += gaussian(x, y, Mona::funcUtils::lerp(minHeight, maxHeight, std::rand() / randMax),
				Mona::funcUtils::lerp(minSigma, maxSigma, std::rand() / randMax),
				{ Mona::funcUtils::lerp(minXY[0], maxXY[0], std::rand()/ randMax),
				Mona::funcUtils::lerp(minXY[1], maxXY[1], std::rand() / randMax) });
		}
		return result;
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
		auto& input = world.GetInput();
		auto& window = world.GetWindow();
		if (input.IsKeyPressed(MONA_KEY_UP))
		{
		}
		else if (input.IsKeyPressed(MONA_KEY_RIGHT))
		{
			m_ikNavHandle->SetAngularSpeed(-m_angularSpeed);
		}
		else if (input.IsKeyPressed(MONA_KEY_LEFT))
		{
			m_ikNavHandle->SetAngularSpeed(m_angularSpeed);
		}
		else {
			m_ikNavHandle->SetAngularSpeed(0);
		}
	};
	virtual void UserStartUp(Mona::World& world) noexcept {
		auto& eventManager = world.GetEventManager();
		eventManager.Subscribe(m_debugGUISubcription, this, &IKRigCharacter::OnDebugGUIEvent);

		m_transform = world.AddComponent<Mona::TransformComponent>(*this);
		m_transform->SetTranslation({ 0,0, 4.5 });
		m_transform->SetScale({ 0.05,0.05,0.05 });
		m_transform->Rotate(glm::vec3(0.0f, 0.0f, 1.0f), glm::radians(180.f));
		m_transform->Rotate(glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(90.f));

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
		m_walkingAnimation = animationManager.LoadAnimationClip(Mona::SourcePath("Assets/Animations/female/walking.fbx"), skeleton, false);
		m_skeletalMesh = world.AddComponent<Mona::SkeletalMeshComponent>(*this, skinnedMesh, m_walkingAnimation, materialPtr);

		Mona::RigData rigData;
		rigData.leftLeg.baseJointName = "Hips";
		rigData.leftLeg.endEffectorName = "LeftFoot";
		rigData.rightLeg.baseJointName = "Hips";
		rigData.rightLeg.endEffectorName = "RightFoot";
		rigData.hipJointName = "Hips";
		m_ikNavHandle = world.AddComponent<Mona::IKNavigationComponent>(*this, rigData);
		world.GetComponentHandle<Mona::IKNavigationComponent>(*this)->AddAnimation(m_walkingAnimation);

	}

	void OnDebugGUIEvent(const Mona::DebugGUIEvent& event) {
		ImGui::Begin("Character Options:");
		ImGui::End();
	}
private:
	float m_angularSpeed = 0.8f;
	bool m_prevIsPress = false;
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
	Mona::GameObjectHandle<Mona::BasicPerspectiveCamera_2> m_camera;
	virtual void UserStartUp(Mona::World &world) noexcept override{
		m_camera = world.CreateGameObject<Mona::BasicPerspectiveCamera_2>();
		world.SetBackgroundColor(0.5, 0.2, 0.8);
		world.SetAmbientLight(glm::vec3(3.8f));
		world.SetMainCamera(world.GetComponentHandle<Mona::CameraComponent>(m_camera));
		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(-130.0f), 2);
		AddDirectionalLight(world, glm::vec3(0.0f, 1.0f, 0.0f), glm::radians(-30.0f), 6);
		auto character = world.CreateGameObject<IKRigCharacter>();
		auto terrainObject = AddTerrain(world);
		world.GetComponentHandle<Mona::IKNavigationComponent>(character)->AddTerrain(terrainObject);
	}

	virtual void UserShutDown(Mona::World& world) noexcept override {
	}
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept override {
		auto& input = world.GetInput();
		auto& window = world.GetWindow();
		if (input.IsKeyPressed(MONA_KEY_ESCAPE)) {
			exit(EXIT_SUCCESS);
		}
		else if (input.IsKeyPressed(MONA_KEY_G))
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
	return 0;
}