#include "World.hpp"
#include "UserHandleTypes.hpp"
#include "../Core/Config.hpp"
#include <glad/glad.h>
#include "../Event/Events.hpp"
#include <chrono>
namespace Mona {
	
	World::World() : 
		m_objectManager(),
		m_eventManager(), 
		m_window(), 
		m_input(), 
		m_application(),
		m_shouldClose(false) {
		m_componentManagers[TransformComponent::componentIndex].reset(new ComponentManager<TransformComponent>());
		m_componentManagers[CameraComponent::componentIndex].reset(new ComponentManager<CameraComponent>());
		m_componentManagers[StaticMeshComponent::componentIndex].reset(new ComponentManager<StaticMeshComponent>());
	}
	void World::StartUp(std::unique_ptr<Application> app) noexcept {
		//m_eventManagerPointer = eventManagerPointer;
		auto& config = Config::GetInstance();
		const GameObjectID expectedObjects = config.getValueOrDefault<int>("expected_number_of_gameobjects", 1000);
		m_window.StartUp(m_eventManager);
		m_input.StartUp(m_eventManager);
		m_objectManager.StartUp(expectedObjects);
		for (auto& componentManager : m_componentManagers)
			componentManager->StartUp(m_eventManager, expectedObjects);
		m_application = std::move(app);
		m_application->StartUp(*this);
	}

	void World::ShutDown() noexcept {
		m_application->UserShutDown(*this);
		m_objectManager.ShutDown(*this);
		for (auto& componentManager : m_componentManagers)
			componentManager->ShutDown(m_eventManager);
		m_window.ShutDown();
		m_input.ShutDown(m_eventManager);
		m_eventManager.ShutDown();

	}

	void World::DestroyGameObject(BaseGameObjectHandle& handle) noexcept {
		DestroyGameObject(*handle);
	}

	void World::DestroyGameObject(GameObject& gameObject) noexcept {
		auto& innerComponentHandles = gameObject.m_componentHandles;
		for (auto& it : innerComponentHandles) {
			m_componentManagers[it.first]->RemoveComponent(it.second);
		}
		innerComponentHandles.clear();
		m_objectManager.DestroyGameObject(*this, gameObject.GetInnerObjectHandle());
	}

	bool World::IsValid(const InnerGameObjectHandle& handle) const noexcept {
		return m_objectManager.IsValid(handle);
	}
	GameObjectManager::size_type World::GetGameObjectCount() const noexcept
	{
		return m_objectManager.GetCount();
	}
	EventManager& World::GetEventManager() noexcept {
		return m_eventManager;
	}

	const Input& World::GetInput() const noexcept {
		return m_input;
	}

	Window& World::GetWindow() noexcept {
		return m_window;
	}

	void World::EndApplication() noexcept {
		m_shouldClose = true;
	}

	void World::StartMainLoop() noexcept {
		const char* vertex_shader_text =
			"#version 450 core\n"
			"void main()\n"
			"{\n"
			"	 const vec4 vertices[6] = vec4[6](vec4(0.25, -0.25, 0.5, 1.0), vec4(-0.25, -0.25, 0.5, 1.0), vec4(0.25, 0.25, 0.5, 1.0), vec4(-0.25, 0.25, 0.5, 1.0), vec4(-0.25, -0.25, 0.5, 1.0), vec4(0.25, 0.25, 0.5, 1.0)); \n"
			"    gl_Position = vertices[gl_VertexID];\n"
			"}\n";

		const char* fragment_shader_text =
			"#version 450 core \n"
			"out vec4 color;\n"
			"void main()\n"
			"{\n"
			"    color = vec4(0.0, 0.8, 1.0, 1.0);\n"
			"}\n";
		GLuint vertex_buffer, vertex_shader, fragment_shader, program;
		GLuint vao;
		glCreateVertexArrays(1, &vao);
		glBindVertexArray(vao);

		vertex_shader = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(vertex_shader, 1, &vertex_shader_text, NULL);
		glCompileShader(vertex_shader);

		fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(fragment_shader, 1, &fragment_shader_text, NULL);
		glCompileShader(fragment_shader);

		program = glCreateProgram();
		glAttachShader(program, vertex_shader);
		glAttachShader(program, fragment_shader);
		glLinkProgram(program);
		glUseProgram(program);

		glm::vec3 pos(1.0f, 1.0f, 2.0f);
		glm::vec3 pos2(2.0f, 3.0f, 4.0f);
		int count = 0;
		float accumMs = 0;
		float accumSec = 0;
		std::chrono::time_point<std::chrono::steady_clock> startTime = std::chrono::steady_clock::now();
		
		while (!m_window.ShouldClose() && !m_shouldClose)
		{
			std::chrono::time_point<std::chrono::steady_clock> newTime = std::chrono::steady_clock::now();
			const auto frameTime = newTime - startTime;
			startTime = newTime;
			float timeStep = std::chrono::duration_cast<std::chrono::duration<float>>(frameTime).count();
			float ms = std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(frameTime).count();
			Update(timeStep);
			accumSec += timeStep;
			accumMs += ms;
			if (count == 200)
			{
				MONA_LOG_INFO("Frame time = {0} ms.\t timeStep = {1} seconds", accumMs/count, accumSec/count);
				count = 0;
				accumMs = 0;
				accumSec = 0;
			}
			count++;
		}
		m_eventManager.Publish(ApplicationEndEvent());
		
	}

	void World::Update(float timeStep) noexcept
	{
		m_input.Update();
		m_objectManager.UpdateGameObjects(*this, timeStep);
		m_application->UserUpdate(*this, timeStep);
		glClear(GL_COLOR_BUFFER_BIT);

		glDrawArrays(GL_TRIANGLES, 0, 6);
		m_window.Update();
	}

}

