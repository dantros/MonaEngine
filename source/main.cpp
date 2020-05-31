
#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#ifndef _WINDOWS_
#undef APIENTRY
#endif
#include "Core/Common.h"
#include "Core/Log.h"
/*
	TODO:
		- Main por ahora hara de engineLoop ------ DONE
		- Add common header => assertions + types (see benny common.hpp).  ------- DONE 
		- Logging class -> Add engine assert/log macros. -------- DONE
		- Create config class. IVAR Registry CVAR config.
		- GLFWWindow.
		- GLFWInput.
		- Window --> typedef?.
		- Input  --> typedef?.
		- memory alloc?
		- Device/Context OpenGL -> benny.
		- Buffer classes -> VBO/IBO/UBO.
		- Shader class -> .
		- VAO.
		- Basic primitives.
		- Textures.
		- Model loading.
		- Mesh drawing.
		- Animation loading.
		- Animation rendering.
		- Collision.
*/
/*
	Reu:
		- Mejor forma de implementar singletons.
		- Engine loop approach.
		- Confirmar plazos sobre plazos.
		- Contar que hare.
		- Framework de testing.
		- 
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

static void error_callback(int error, const char* description)
{
	fprintf(stderr, "Error: %s\n", description);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GLFW_TRUE);
}

int main()
{
	GLFWwindow* window;
	GLuint vertex_buffer, vertex_shader, fragment_shader, program;
	
	Mona::Log::StartUp();

	glfwSetErrorCallback(error_callback);

	if (!glfwInit())
		exit(EXIT_FAILURE);

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	const char* title = "Simple example";
	glm::vec<2,int> windowDimensions(1200, 600);
	window = glfwCreateWindow(windowDimensions.x, windowDimensions.y, title, NULL, NULL);
	MONA_LOG_INFO("Creating GLFW window with title: {0}, and dimensions ({1},{2})", title, windowDimensions.x, windowDimensions.y);

	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	
	glfwSetKeyCallback(window, key_callback);

	glfwMakeContextCurrent(window);
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwSwapInterval(1);

	// NOTE: OpenGL error checks have been omitted for brevity
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
	while (!glfwWindowShouldClose(window))
	{
		float ratio;
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		ratio = width / (float)height;
		glViewport(0, 0, width, height);
		glClear(GL_COLOR_BUFFER_BIT);
		glUseProgram(program);
		glDrawArrays(GL_TRIANGLES, 0, 6);
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwDestroyWindow(window);
	glfwTerminate();
	exit(EXIT_SUCCESS);
}