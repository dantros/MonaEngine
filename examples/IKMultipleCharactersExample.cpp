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
	world.AddComponent<Mona::DirectionalLightComponent>(light, lightIntensity * glm::vec3(1.0f));

}

Mona::GameObjectHandle<Mona::GameObject> AddTerrain(Mona::World& world) {
	auto terrain = world.CreateGameObject<Mona::GameObject>();
	auto& meshManager = Mona::MeshManager::GetInstance();
	auto materialPtr = std::static_pointer_cast<Mona::DiffuseFlatMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseFlat));
	materialPtr->SetDiffuseColor(glm::vec3(0.3, 0.1f, 0.1f));
	auto transform = world.AddComponent<Mona::TransformComponent>(terrain);
	transform->SetTranslation(glm::vec3(0, 0, 0));
	glm::vec2 minXY(-100, -100);
	glm::vec2 maxXY(100, 100);
	int numInnerVerticesWidth = 100;
	int numInnerVerticesHeight = 100;
	auto heighFunc = [](float x, float y) -> float {
		float result = 0;
		int funcNum = 200;
		glm::vec2 minXY(-100, -100);
		glm::vec2 maxXY(100, 100);
		float minHeight = -15;
		float maxHeight = 70;
		float minSigma = 3;
		float maxSigma = 15;
		std::srand(5);
		for (int i = 0; i < funcNum; i++) {
			float randMax = RAND_MAX;
			result += gaussian(x, y, Mona::funcUtils::lerp(minHeight, maxHeight, std::rand() / randMax),
				Mona::funcUtils::lerp(minSigma, maxSigma, std::rand() / randMax),
				{ Mona::funcUtils::lerp(minXY[0], maxXY[0], std::rand() / randMax),
				Mona::funcUtils::lerp(minXY[1], maxXY[1], std::rand() / randMax) });
		}
		return result;
	};

	world.AddComponent<Mona::StaticMeshComponent>(terrain, meshManager.GenerateTerrain(minXY, maxXY, numInnerVerticesWidth,
		numInnerVerticesHeight, heighFunc), materialPtr);
	return terrain;
}


class IKRigCharacter : public Mona::GameObject
{
private:

	void UpdateAnimationState() {
		auto& animController = m_skeletalMesh->GetAnimationController();


	}
public:
	IKRigCharacter(std::string characterName, glm::vec3 startingPosition, int walkingAnimIndex) {
		m_characterName = characterName;
		m_startingPosition = startingPosition;
		m_walkingAnimIndex = walkingAnimIndex;
	}
	virtual void UserUpdate(Mona::World& world, float timeStep) noexcept {
		UpdateAnimationState();
		auto& input = world.GetInput();
		auto& window = world.GetWindow();
		auto& animController = m_skeletalMesh->GetAnimationController();
		if (input.IsKeyPressed(MONA_KEY_UP))
		{
			Mona::BlendType blendType = m_walkingAnimation == animController.GetCurrentAnimation() ? Mona::BlendType::KeepSynchronize : Mona::BlendType::Smooth;
			animController.FadeTo(m_walkingAnimation, blendType, m_fadeTime, 0.0f);
		}
		else if (input.IsKeyPressed(MONA_KEY_DOWN)) {
			animController.FadeTo(m_idleAnimation, Mona::BlendType::Smooth, m_fadeTime, 0.0f);
		}

		if (input.IsKeyPressed(MONA_KEY_RIGHT))
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

		world.GetComponentHandle<Mona::IKNavigationComponent>(*this)->SetStrideCorrection(m_correctStrides);
		world.GetComponentHandle<Mona::IKNavigationComponent>(*this)->SetStrideValidation(m_validateStrides);
		world.GetComponentHandle<Mona::IKNavigationComponent>(*this)->EnableIK(m_enableIK);

	};
	virtual void UserStartUp(Mona::World& world) noexcept {
		auto& eventManager = world.GetEventManager();
		eventManager.Subscribe(m_debugGUISubcription, this, &IKRigCharacter::OnDebugGUIEvent);

		m_transform = world.AddComponent<Mona::TransformComponent>(*this);

		auto materialPtr = std::static_pointer_cast<Mona::DiffuseTexturedMaterial>(world.CreateMaterial(Mona::MaterialType::DiffuseTextured, true));
		auto& textureManager = Mona::TextureManager::GetInstance();
		auto diffuseTexture = textureManager.LoadTexture(Mona::SourcePath("Assets/Textures/" + m_characterName + "/diffuse.png"));
		materialPtr->SetMaterialTint(glm::vec3(0.1f));
		materialPtr->SetDiffuseTexture(diffuseTexture);

		auto& meshManager = Mona::MeshManager::GetInstance();
		auto& skeletonManager = Mona::SkeletonManager::GetInstance();
		auto& animationManager = Mona::AnimationClipManager::GetInstance();
		auto skeleton = skeletonManager.LoadSkeleton(Mona::SourcePath("Assets/Models/" + m_characterName + ".fbx"));
		auto skinnedMesh = meshManager.LoadSkinnedMesh(skeleton, Mona::SourcePath("Assets/Models/" + m_characterName + ".fbx"), true);

		m_idleAnimation = animationManager.LoadAnimationClip(Mona::SourcePath("Assets/Animations/" + m_characterName + "/idle.fbx"), skeleton, true);
		m_walkingAnimation = animationManager.LoadAnimationClip(Mona::SourcePath("Assets/Animations/" + m_characterName + "/walking"
			+ std::to_string(m_walkingAnimIndex) + ".fbx"), skeleton, false);

		m_skeletalMesh = world.AddComponent<Mona::SkeletalMeshComponent>(*this, skinnedMesh, m_idleAnimation, materialPtr);

		Mona::RigData rigData;
		rigData.leftLeg.baseJointName = "mixamorig:LeftUpLeg";
		rigData.leftLeg.endEffectorName = "mixamorig:LeftFoot";
		rigData.rightLeg.baseJointName = "mixamorig:RightUpLeg";
		rigData.rightLeg.endEffectorName = "mixamorig:RightFoot";
		rigData.hipJointName = "mixamorig:Hips";
		rigData.initialRotationAngle = 0.0f;
		rigData.initialPosition = m_startingPosition;
		rigData.scale = 0.05f;

		glm::vec3 originalUpVector = glm::vec3(0, 1, 0);
		glm::vec3 originalFrontVector = glm::vec3(0, 0, 1);
		m_ikNavHandle = world.AddComponent<Mona::IKNavigationComponent>(*this, rigData);
		world.GetComponentHandle<Mona::IKNavigationComponent>(*this)->AddAnimation(m_walkingAnimation, originalUpVector,
			originalFrontVector, Mona::AnimationType::WALKING);
		world.GetComponentHandle<Mona::IKNavigationComponent>(*this)->AddAnimation(m_idleAnimation, originalUpVector,
			originalFrontVector, Mona::AnimationType::IDLE);
		m_skeletalMesh->GetAnimationController().SetPlayRate(m_playRate);

	}
	void OnDebugGUIEvent(const Mona::DebugGUIEvent& event) {
		ImGui::Begin("IKNav Options:");
		ImGui::Checkbox("Validate strides", &(m_validateStrides));
		ImGui::Checkbox("Correct strides", &(m_correctStrides));
		ImGui::Checkbox("Enable IK", &(m_enableIK));
		ImGui::End();
	}
private:
	float m_angularSpeed = 0.8f;
	float m_fadeTime = 0.5f;
	bool m_validateStrides = false;
	bool m_correctStrides = true;
	bool m_enableIK = true;
	float m_playRate = 0.7f;
	int m_walkingAnimIndex;
	std::string m_characterName;
	glm::vec3 m_startingPosition;
	Mona::TransformHandle m_transform;
	Mona::SkeletalMeshHandle m_skeletalMesh;
	Mona::IKNavigationHandle m_ikNavHandle;
	std::shared_ptr<Mona::AnimationClip> m_walkingAnimation;
	std::shared_ptr<Mona::AnimationClip> m_idleAnimation;
	Mona::SubscriptionHandle m_debugGUISubcription;

};

class IKNav : public Mona::Application
{
public:
	IKNav() = default;
	~IKNav() = default;
	Mona::GameObjectHandle<Mona::BasicPerspectiveCamera_2> m_camera;
	virtual void UserStartUp(Mona::World& world) noexcept override {
		m_camera = world.CreateGameObject<Mona::BasicPerspectiveCamera_2>();
		world.SetBackgroundColor(0.5, 0.2, 0.8);
		world.SetAmbientLight(glm::vec3(4.0f));
		world.SetMainCamera(world.GetComponentHandle<Mona::CameraComponent>(m_camera));
		AddDirectionalLight(world, glm::vec3(1.0f, 0.0f, 0.0f), glm::radians(-130.0f), 2);
		AddDirectionalLight(world, glm::vec3(0.0f, 1.0f, 0.0f), glm::radians(-30.0f), 8.5f);
		auto terrainObject1 = AddTerrain(world);
		// choosable characters: ely, sportyGranny, prisoner, maria, theBoss, xbot.
		// choosable walking animations: 0, 1 ,2 .
		auto character1 = world.CreateGameObject<IKRigCharacter>("ely",glm::vec3(0,0,0), 1);
		world.GetComponentHandle<Mona::IKNavigationComponent>(character1)->AddTerrain(terrainObject1);
		auto character2 = world.CreateGameObject<IKRigCharacter>("maria", glm::vec3(-20, -5, 0), 2);
		world.GetComponentHandle<Mona::IKNavigationComponent>(character2)->AddTerrain(terrainObject1);
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