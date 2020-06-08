

#include "Core/Common.hpp"
#include "Core/Log.hpp"
#include "Core/Config.hpp"
#include <glad/glad.h>
#include "Platform/Window.hpp"
#include "Platform/Input.hpp"


/*
	TODO:
		- GLFWWindow.
			- Complete creation. ---- DONE
			- Complete destruction. ----- DONE
			- Complete windowMode => Fullscreen/NoFullscreen. ----- DONE
			- Complete windowRes. ----------------- DONE 
			- Default callbacks.
				- ERRROR. ------------- DONE
		- GLFWInput. ------------- DONE
		- Events for both
		- memory alloc?
		- Device/Context OpenGL -> benny.
		- Buffer classes -> VBO/IBO/UBO.
		- Shader class -> .
		- VAO.
		- Basic primitives.
		- Textures.
		- Model loading.
		- Resource Manager
		- Mesh drawing.
		- Animation loading.
		- Animation rendering.
		- Collision.
*/
/*
	Reu:
		- Temas memoria
			- Contar lo que se espera de la segunda entrega.
			- Preguntar cual debería ser mi acercamiento al Estado del arte.
			- Confirmar plazos.
		- Temas codigo.
			- Mejor forma de implementar singletons.
			- Engine loop approach.
			- Contar en que estoy hasta el momento.
			- Framework de testing.

*/

#include <iostream>
#include <glm/glm.hpp>
#include <memory>

static const char* vertex_shader_text =
"#version 450 core\n"
"void main()\n"
"{\n"
"	 const vec4 vertices[6] = vec4[6](vec4(0.25, -0.25, 0.5, 1.0), vec4(-0.25, -0.25, 0.5, 1.0), vec4(0.25, 0.25, 0.5, 1.0), vec4(-0.25, 0.25, 0.5, 1.0), vec4(-0.25, -0.25, 0.5, 1.0), vec4(0.25, 0.25, 0.5, 1.0)); \n"
"    gl_Position = vertices[gl_VertexID];\n"
"}\n";

static const char* fragment_shader_text =
"#version 450 core \n"
"out vec4 color;\n"
"void main()\n"
"{\n"
"    color = vec4(0.0, 0.8, 1.0, 1.0);\n"
"}\n";

int main()
{
	GLuint vertex_buffer, vertex_shader, fragment_shader, program;
	
	Mona::Log::StartUp();
	Mona::Config& config = Mona::Config::GetInstance();
	config.readFile("config.cfg");
	Mona::Window window = Mona::Window();
	window.StartUp();
	Mona::Input input = Mona::Input();
	

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
	while (!window.ShouldClose())
	{

		input.Update();
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

		if (count > 120)
		{
			auto cursorPos = input.GetMousePosition();
			MONA_LOG_INFO("The is cursor is at ({0},{1})", cursorPos.x, cursorPos.y);
			count = 0;
		}
		glClear(GL_COLOR_BUFFER_BIT);
		glUseProgram(program);
		glDrawArrays(GL_TRIANGLES, 0, 6);
		window.Update();
		count++;
	}

	window.ShutDown();
}