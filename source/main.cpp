
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
#include <thread>
#include <mutex>
#include <vector>
#include <condition_variable>

std::mutex m;
std::condition_variable cv;
struct rendercommand{
	GLint first;
	GLsizei count;
};
std::vector<rendercommand> data;
bool ready = false;
bool processed = false;
bool running = true;

void worker_thread()
{
	while (running)
	{
		// Wait until main() sends data
		std::unique_lock<std::mutex> lk(m);
		cv.wait(lk, [] {return ready; });

		// after the wait, we own the lock.
		data.push_back({ 0, 3 });
		data.push_back({ 3, 3 });

		// Send data back to main()
		processed = true;
		// Manual unlocking is done before notifying, to avoid waking up
		// the waiting thread only to block again (see notify_one for details)
		ready = false;
		lk.unlock();
		cv.notify_one();

	}
	
}

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
	spdlog::set_pattern("%^[%T] %n: %v%$");

	auto s_coreLogger = spdlog::stdout_color_mt("MONA");
	s_coreLogger->set_level(spdlog::level::trace);
	
	auto s_clientLogger = spdlog::stdout_color_mt("APP");
	s_clientLogger->set_level(spdlog::level::trace);

	
	glfwSetErrorCallback(error_callback);

	if (!glfwInit())
		exit(EXIT_FAILURE);

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

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
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		s_coreLogger->error("Failed to load OpenGL functions using glad");
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
	std::thread worker(worker_thread);
	while (!glfwWindowShouldClose(window))
	{
		float ratio;
		int width, height;
		

		glfwGetFramebufferSize(window, &width, &height);
		ratio = width / (float)height;

		glViewport(0, 0, width, height);
		glClear(GL_COLOR_BUFFER_BIT);

		

		glUseProgram(program);
		{
			std::lock_guard<std::mutex> lk(m);
			data.clear();
			ready = true;
			
		}
		cv.notify_one();

		// wait for the worker
		{
			std::unique_lock<std::mutex> lk(m);
			cv.wait(lk, [] {return processed; });
			processed = false;
		}
		for (const auto& command : data)
		{
			glDrawArrays(GL_TRIANGLES, command.first, command.count);
		}

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwDestroyWindow(window);

	glfwTerminate();
	exit(EXIT_SUCCESS);
}