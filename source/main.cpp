
#include <glad/glad.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#ifndef _WINDOWS_
#undef APIENTRY
#endif

#include "spdlog/spdlog.h"
#include <spdlog/sinks/stdout_color_sinks.h>
#include <iostream>
#include <glm/glm.hpp>

static const char* vertex_shader_text =
"#version 460\n"
"void main()\n"
"{\n"
"	 const vec4 vertices[3] = vec4[3](vec4(0.25, -0.25, 0.5, 1.0), vec4(-0.25, -0.25, 0.5, 1.0), vec4(0.25, 0.25, 0.5, 1.0)); \n"
"    gl_Position = vertices[gl_VertexID];\n"
"}\n";

static const char* fragment_shader_text =
"#version 460\n"
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
	spdlog::set_pattern("%^[%T] %n: %v%$");

	auto s_coreLogger = spdlog::stdout_color_mt("MONA");
	s_coreLogger->set_level(spdlog::level::trace);
	
	auto s_clientLogger = spdlog::stdout_color_mt("APP");
	s_clientLogger->set_level(spdlog::level::trace);

	
	glfwSetErrorCallback(error_callback);

	if (!glfwInit())
		exit(EXIT_FAILURE);

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);

	const char* title = "Simple example";
	glm::vec<2,int> windowDimensions(1200, 600);

	s_coreLogger->info("Creating GLFW Window {0} with dimensions ({1},{2})", title, windowDimensions.x, windowDimensions.y);
	window = glfwCreateWindow(windowDimensions.x, windowDimensions.y, title, NULL, NULL);
	if (!window)
	{
		glfwTerminate();
		exit(EXIT_FAILURE);
	}

	glfwSetKeyCallback(window, key_callback);

	glfwMakeContextCurrent(window);
	gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
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
		glDrawArrays(GL_TRIANGLES, 0, 3);

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwDestroyWindow(window);

	glfwTerminate();
	exit(EXIT_SUCCESS);
}