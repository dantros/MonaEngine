#include "Engine.hpp"
#include "Core/Common.hpp"
#include "Core/Log.hpp"
#include "Core/Config.hpp"
#include <glad/glad.h>
#include "Event/Events.hpp"

namespace Mona {
	
	void Engine::StartUp() noexcept
	{
		Mona::Log::StartUp();
		Mona::Config& config = Mona::Config::GetInstance();
		config.readFile("config.cfg");
		m_window.StartUp(&m_eventManager);
		m_input.StartUp(&m_eventManager);
		
	}

	void Engine::ShutDown() noexcept
	{
		//First thing to clean should be Event Manager.
		m_window.ShutDown();
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
		while (!m_window.ShouldClose())
		{
			m_input.Update();
			if (m_input.IsKeyPressed(MONA_KEY_G))
			{
				m_window.SetFullScreen(true);
			}
			else if (m_input.IsKeyPressed(MONA_KEY_H))
			{
				m_window.SetFullScreen(false);
			}
			else if (m_input.IsKeyPressed(MONA_KEY_J))
			{
				m_window.SetWindowDimensions(glm::ivec2(1000, 1000));
			}
			
			if (count > 360)
			{
				auto cursorPos = m_input.GetMousePosition();
				MONA_LOG_INFO("The is cursor is at ({0},{1})", cursorPos.x, cursorPos.y);
				count = 0;
			}
			if (m_input.GetMouseWheelOffset().y > 0.0)
			{
				auto Offset = m_input.GetMouseWheelOffset();
				MONA_LOG_INFO("The mouse offset is ({0},{1})", Offset.x, Offset.y);
				
			}
			glClear(GL_COLOR_BUFFER_BIT);
			glUseProgram(program);
			glDrawArrays(GL_TRIANGLES, 0, 6);
			m_window.Update();
			count++;
		}

	}

}