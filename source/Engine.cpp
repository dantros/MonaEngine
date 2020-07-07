#include "Engine.hpp"
#include "Core/Common.hpp"
#include "Core/Log.hpp"
#include "Core/Config.hpp"
#include <glad/glad.h>
#include "Event/Events.hpp"
#include "World/World.hpp"
#include <chrono>

namespace Mona {
	
	void Engine::StartUp(std::unique_ptr<Application> app) noexcept
	{
		
		Mona::Log::StartUp();
		Mona::Config& config = Mona::Config::GetInstance();
		config.readFile("config.cfg");
		MONA_ASSERT(app != nullptr, "Engine StartUp Error: Must provido not null Application pointer");
		m_window = std::make_shared<Window>();
		m_input = std::make_shared<Input>();
		m_window->StartUp(&m_eventManager);
		m_input->StartUp(&m_eventManager);
		m_application = std::move(app);
		m_application->StartUp(m_window, m_input);
		Mona::World::GetInstance().StartUp();
		
	}

	void Engine::ShutDown() noexcept
	{
		//First thing to clean should be Event Manager.
		m_application->UserShutDown();
		Mona::World::GetInstance().ShutDown();
		m_window->ShutDown();

	}

	void Engine::StartMainLoop() noexcept
	{
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


		glm::vec3 pos(1.0f, 1.0f, 2.0f);
		glm::vec3 pos2(2.0f, 3.0f, 4.0f);
		int count = 0;
		float accumMs = 0;
		float accumSec = 0;
		std::chrono::time_point<std::chrono::steady_clock> startTime = std::chrono::steady_clock::now();
		auto& world = Mona::World::GetInstance();
		while (!m_window->ShouldClose())
		{
			std::chrono::time_point<std::chrono::steady_clock> newTime = std::chrono::steady_clock::now();
			const auto frameTime = newTime - startTime;
			startTime = newTime;
			float timeStep = std::chrono::duration_cast<std::chrono::duration<float>>(frameTime).count();
			float ms = std::chrono::duration_cast<std::chrono::duration<float, std::milli>>(frameTime).count();
			
			world.Update(timeStep);
			m_application->UserUpdate(timeStep);
			accumSec += timeStep;
			accumMs += ms;
			if (count == 200)
			{
				MONA_LOG_INFO("Frame time = {0} ms.\t timeStep = {1} seconds", accumMs/count, accumSec/count);
				count = 0;
				accumMs = 0;
				accumSec = 0;
			}
			
			glClear(GL_COLOR_BUFFER_BIT);
			glUseProgram(program);
			glDrawArrays(GL_TRIANGLES, 0, 6);
			m_window->Update();
			m_input->Update();
			count++;
		}

	}

}