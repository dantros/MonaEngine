set(MONA_HEADERS 
				Core/Common.hpp 
				Core/Log.hpp
				Core/Config.hpp
				Core/RootDirectory.hpp
				Core/AssimpTransformations.hpp
				Platform/Window.hpp
				Platform/Input.hpp
				Platform/KeyCodes.hpp
				Event/EventManager.hpp
				Event/Events.hpp
				Engine.hpp
				Application.hpp
				World/GameObjectTypes.hpp
				World/GameObject.hpp
				World/GameObjectManager.hpp
				World/Detail/GameObjectManager_Implementation.hpp
				World/TransformComponent.hpp
				World/ComponentTypes.hpp
				World/ComponentManager.hpp
				World/Detail/ComponentManager_Implementation.hpp
				World/World.hpp
				World/ComponentHandle.hpp
				World/GameObjectHandle.hpp
				World/Detail/World_Implementation.hpp
				Rendering/Renderer.hpp
				Rendering/CameraComponent.hpp
				Rendering/StaticMeshComponent.hpp
				Rendering/ShaderProgram.hpp
				Rendering/MeshManager.hpp
				Rendering/Mesh.hpp
				Rendering/Material.hpp
				Rendering/Texture.hpp
				Rendering/TextureManager.hpp
				Rendering/UnlitFlatMaterial.hpp
				Rendering/UnlitTexturedMaterial.hpp
				Rendering/DiffuseFlatMaterial.hpp
				Rendering/DiffuseTexturedMaterial.hpp
				Rendering/PBRTexturedMaterial.hpp
				Rendering/PBRFlatMaterial.hpp
				Rendering/DirectionalLightComponent.hpp
				Rendering/SpotLightComponent.hpp
				Rendering/PointLightComponent.hpp
				Animation/JointPose.hpp
				Animation/AnimationClip.hpp
				Animation/Skeleton.hpp
				Animation/SkeletalMeshComponent.hpp
				Animation/SkeletonManager.hpp
				Animation/SkinnedMesh.hpp
				Animation/AnimationClipManager.hpp
				Animation/AnimationSystem.hpp
				Animation/CrossFadeTarget.hpp
				Animation/AnimationController.hpp
				PhysicsCollision/PhysicsCollisionSystem.hpp
				PhysicsCollision/PhysicsCollisionEvents.hpp
				PhysicsCollision/CustomMotionState.hpp
				PhysicsCollision/RigidBodyComponent.hpp
				PhysicsCollision/ShapeTypes.hpp
				PhysicsCollision/RigidBodyLifetimePolicy.hpp
				PhysicsCollision/RaycastResults.hpp
				PhysicsCollision/CollisionInformation.hpp
				Audio/AudioSystem.hpp
				Audio/AudioMacros.hpp
				Audio/AudioClip.hpp
				Audio/AudioClipManager.hpp
				Audio/AudioSource.hpp
				Audio/AudioSourceComponent.hpp
				Audio/AudioSourceComponentLifetimePolicy.hpp
				DebugDrawing/DebugDrawingSystem.hpp
				DebugDrawing/BulletDebugDraw.hpp
				Utilities/BasicCameraControllers.hpp)
set(MONA_SOURCES 
				Core/Config.cpp
				Event/EventManager.cpp
				Platform/Window.cpp
				Platform/Input.cpp
				Application.cpp
				World/GameObjectManager.cpp
				World/World.cpp
				Rendering/Renderer.cpp
				Rendering/ShaderProgram.cpp
				Rendering/MeshManager.cpp
				Rendering/Texture.cpp
				Rendering/TextureManager.cpp
				Rendering/Mesh.cpp
				Animation/AnimationClipManager.cpp
				Animation/SkeletonManager.cpp
				Animation/AnimationSystem.cpp
				Animation/AnimationClip.cpp
				Animation/AnimationController.cpp
				Animation/SkinnedMesh.cpp
				Animation/Skeleton.cpp
				PhysicsCollision/PhysicsCollisionSystem.cpp
				Audio/AudioSystem.cpp
				Audio/AudioClip.cpp
				Audio/AudioClipManager.cpp
				Audio/AudioSourceComponent.cpp
				DebugDrawing/ImGuiBuild.cpp
				DebugDrawing/BulletDebugDraw.cpp
				DebugDrawing/DebugDrawingSystem.cpp)
file(GLOB_RECURSE Shaders  "${CMAKE_SOURCE_DIR}/Assets/Shaders/*")
source_group("Shaders" FILES ${Shaders})

add_library(MonaEngine STATIC ${MONA_SOURCES} ${MONA_HEADERS} MonaEngine.hpp ${Shaders})
if (MSVC)
    target_compile_options(MonaEngine PUBLIC /wd5033)
endif(MSVC)
target_include_directories(MonaEngine PRIVATE ${THIRD_PARTY_INCLUDE_DIRECTORIES} MONA_INCLUDE_DIRECTORY)
target_link_libraries(MonaEngine PRIVATE ${THIRD_PARTY_LIBRARIES})
set_property(TARGET MonaEngine PROPERTY CXX_STANDARD 20)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${MONA_SOURCES} ${MONA_HEADERS})


