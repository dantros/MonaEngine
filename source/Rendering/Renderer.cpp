#include "Renderer.hpp"
#include <imgui.h>
#include "examples/imgui_impl_glfw.h"
#include "examples/imgui_impl_opengl3.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/type_ptr.hpp>
#include "../Core/Log.hpp"
void GLAPIENTRY MessageCallback(GLenum source,
								GLenum type,
								GLuint id,
								GLenum severity,
								GLsizei length,
								const GLchar* message,
								const void* userParam)
{
	MONA_LOG_ERROR("OpenGL Error: type = {0}, message = {1}",type,  message);
		
}


namespace Mona{
	template
		class ComponentManager<CameraComponent>;

	template
		class ComponentManager<TransformComponent>;
	template
		class ComponentManager<StaticMeshComponent>;



	void Renderer::StartUp(EventManager& eventManager) noexcept {
		const char* vertex_shader_text =
			"#version 450 core\n"
			"layout(location = 3) uniform mat4 viewMatrix;\n"
			"layout(location = 4) uniform mat4 projectionMatrix;\n"
			"layout(location = 5) uniform mat4 modelMatrix;\n"
			"out vec3 normal; \n"
			"void main()\n"
			"{\n"
			"vec3[36] vertices = vec3[36]( \n"
			"vec3(-0.5f, -0.5f, -0.5f),  \n"
			"vec3( 0.5f, -0.5f, -0.5f),  \n"
			"vec3( 0.5f,  0.5f, -0.5f),  \n"
			"vec3( 0.5f,  0.5f, -0.5f),  \n"
			"vec3(-0.5f,  0.5f, -0.5f),  \n"
			"vec3(-0.5f, -0.5f, -0.5f),  \n"
			"vec3(-0.5f, -0.5f,  0.5f),  \n"
			"vec3( 0.5f, -0.5f,  0.5f),  \n"
			"vec3( 0.5f,  0.5f,  0.5f),  \n"
			"vec3( 0.5f,  0.5f,  0.5f),  \n"
			"vec3(-0.5f,  0.5f,  0.5f),  \n"
			"vec3(-0.5f, -0.5f,  0.5f),  \n"
			"vec3(-0.5f,  0.5f,  0.5f),  \n"
			"vec3(-0.5f,  0.5f, -0.5f),  \n"
			"vec3(-0.5f, -0.5f, -0.5f),  \n"
			"vec3(-0.5f, -0.5f, -0.5f),  \n"
			"vec3(-0.5f, -0.5f,  0.5f),  \n"
			"vec3(-0.5f,  0.5f,  0.5f),  \n"
			"vec3( 0.5f,  0.5f,  0.5f),  \n"
			"vec3( 0.5f,  0.5f, -0.5f),  \n"
			"vec3( 0.5f, -0.5f, -0.5f),  \n"
			"vec3( 0.5f, -0.5f, -0.5f),  \n"
			"vec3( 0.5f, -0.5f,  0.5f),  \n"
			"vec3( 0.5f,  0.5f,  0.5f),  \n"
			"vec3(-0.5f, -0.5f, -0.5f),  \n"
			"vec3( 0.5f, -0.5f, -0.5f),  \n"
			"vec3( 0.5f, -0.5f,  0.5f),  \n"
			"vec3( 0.5f, -0.5f,  0.5f),  \n"
			"vec3(-0.5f, -0.5f,  0.5f),  \n"
			"vec3(-0.5f, -0.5f, -0.5f),  \n"
			"vec3(-0.5f,  0.5f, -0.5f),  \n"
			"vec3( 0.5f,  0.5f, -0.5f),  \n"
			"vec3( 0.5f,  0.5f,  0.5f),  \n"
			"vec3( 0.5f,  0.5f,  0.5f),  \n"
			"vec3(-0.5f,  0.5f,  0.5f),  \n"
			"vec3(-0.5f,  0.5f, -0.5f));\n"
			"vec3[36] normals =  vec3[36]( \n"
			"vec3( 0.0f,  0.0f, -1.0f), \n"
			"vec3( 0.0f,  0.0f, -1.0f), \n"
			"vec3( 0.0f,  0.0f, -1.0f), \n"
			"vec3( 0.0f,  0.0f, -1.0f), \n"
			"vec3( 0.0f,  0.0f, -1.0f), \n"
			"vec3( 0.0f,  0.0f, -1.0f), \n"
			"vec3( 0.0f,  0.0f,  1.0f), \n"
			"vec3( 0.0f,  0.0f,  1.0f), \n"
			"vec3( 0.0f,  0.0f,  1.0f), \n"
			"vec3( 0.0f,  0.0f,  1.0f), \n"
			"vec3( 0.0f,  0.0f,  1.0f), \n"
			"vec3( 0.0f,  0.0f,  1.0f), \n"
			"vec3(-1.0f,  0.0f,  0.0f), \n"
			"vec3(-1.0f,  0.0f,  0.0f), \n"
			"vec3(-1.0f,  0.0f,  0.0f), \n"
			"vec3(-1.0f,  0.0f,  0.0f), \n"
			"vec3(-1.0f,  0.0f,  0.0f), \n"
			"vec3(-1.0f,  0.0f,  0.0f), \n"
			"vec3( 1.0f,  0.0f,  0.0f), \n"
			"vec3( 1.0f,  0.0f,  0.0f), \n"
			"vec3( 1.0f,  0.0f,  0.0f), \n"
			"vec3( 1.0f,  0.0f,  0.0f), \n"
			"vec3( 1.0f,  0.0f,  0.0f), \n"
			"vec3( 1.0f,  0.0f,  0.0f), \n"
			"vec3( 0.0f, -1.0f,  0.0f), \n"
			"vec3( 0.0f, -1.0f,  0.0f), \n"
			"vec3( 0.0f, -1.0f,  0.0f), \n"
			"vec3( 0.0f, -1.0f,  0.0f), \n"
			"vec3( 0.0f, -1.0f,  0.0f), \n"
			"vec3( 0.0f, -1.0f,  0.0f), \n"
			"vec3( 0.0f,  1.0f,  0.0f), \n"
			"vec3( 0.0f,  1.0f,  0.0f), \n"
			"vec3( 0.0f,  1.0f,  0.0f), \n"
			"vec3( 0.0f,  1.0f,  0.0f), \n"
			"vec3( 0.0f,  1.0f,  0.0f), \n"
			"vec3( 0.0f,  1.0f,  0.0f)); \n"
			"normal = mat3(transpose(inverse(modelMatrix))) * normals[gl_VertexID];"
			"gl_Position = projectionMatrix * viewMatrix *modelMatrix * (vec4(1.5,1.0,1.0,1.0) * vec4(vertices[gl_VertexID],1.0)); \n"
			
			"}\n";


		const char* fragment_shader_text =
			"#version 450 core \n"
			"out vec4 color;\n"
			"in vec3 normal;\n"
			"void main()\n"
			"{\n"
			" vec3 objectColor = vec3(0.2,0.2,0.8);"
			" float ambient = 0.2f;"
			" vec3 norm = normalize(normal);\n"
			" vec3 lightDir = normalize(vec3(-1.0,1.0,-2.0));\n"
			" float diffuse = max(dot(norm, -lightDir), 0.0); \n"
			" vec3 result = (ambient + diffuse) * objectColor; \n"
			" color = vec4(result, 1.0);\n"
			"}\n";
		GLuint vertex_buffer, vertex_shader, fragment_shader, program;
		GLuint vao;
		glCreateVertexArrays(1, &vao);
		glBindVertexArray(vao);

		vertex_shader = glCreateShader(GL_VERTEX_SHADER);
		glShaderSource(vertex_shader, 1, &vertex_shader_text, 0);
		glCompileShader(vertex_shader);

		fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
		glShaderSource(fragment_shader, 1, &fragment_shader_text, 0);
		glCompileShader(fragment_shader);

		program = glCreateProgram();
		glAttachShader(program, vertex_shader);
		glAttachShader(program, fragment_shader);
		glLinkProgram(program);
		glUseProgram(program);
		m_onWindowResizeSubscription = eventManager.Subscribe(this, &Renderer::OnWindowResizeEvent);

		glEnable(GL_DEPTH_TEST);
		StartDebugConfiguration();
		StartImGui();
	}
	void Renderer::ShutDown(EventManager& eventManager) noexcept {
		eventManager.Unsubscribe(m_onWindowResizeSubscription);
	}
	void Renderer::OnWindowResizeEvent(const WindowResizeEvent& event) {
		if (event.width == 0 || event.height == 0)
			return;
		glViewport(0, 0, event.width, event.height);
	}

	void Renderer::StartDebugConfiguration() noexcept {
		glEnable(GL_DEBUG_OUTPUT);
		glDebugMessageCallback(MessageCallback, 0);
		glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_NOTIFICATION, 0, nullptr, GL_FALSE);
	}

	void Renderer::StartImGui() noexcept {

		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImGuiIO& io = ImGui::GetIO(); (void)io;
		ImGui::StyleColorsDark();
		ImGui_ImplGlfw_InitForOpenGL(glfwGetCurrentContext(), true);
		ImGui_ImplOpenGL3_Init("#version 450");

	}

	void Renderer::RenderImGui(EventManager& eventManager) noexcept {
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		{
			ImGui::Begin("FPS Counter:");
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::End();
		}
		eventManager.Publish(DebugGUIEvent());
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	}

	void Renderer::Render(EventManager& eventManager,
				InnerComponentHandle cameraHandle,
				ComponentManager<StaticMeshComponent>& staticMeshDataManager,
				ComponentManager<TransformComponent>& transformDataManager,
				ComponentManager<CameraComponent>& cameraDataManager) noexcept 
	{
		//Add depth testing face culling in the future
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//Setting Scene Data

		//TODO(BYRON) : This should be replace with getting cameraComponentfrom code.

		//CameraComponent& camera = cameraDataManager[0];
		glm::mat4 viewMatrix;
		glm::mat4 projectionMatrix;
		if (cameraDataManager.IsValid(cameraHandle)) {
			CameraComponent* camera = cameraDataManager.GetComponentPointer(cameraHandle);
			GameObject* cameraOwner = cameraDataManager.GetOwner(cameraHandle);
			TransformComponent* cameraTransform = transformDataManager.GetComponentPointer(cameraOwner->GetInnerComponentHandle<TransformComponent>());
			viewMatrix = cameraTransform->GetViewMatrixFromTransform();
			projectionMatrix = camera->GetProjectionMatrix();
		}
		else {
			MONA_LOG_INFO("Render Info: No camera has been set, using defaults transformations");
			viewMatrix = glm::mat4(1.0f);
			projectionMatrix = glm::perspective(glm::radians(50.0f), 16.0f / 9.0f, 0.1f, 100.0f);
		}
		glUniformMatrix4fv(3, 1, GL_FALSE, glm::value_ptr(viewMatrix));
		glUniformMatrix4fv(4, 1, GL_FALSE, glm::value_ptr(projectionMatrix));
		//Iterating over each static mesh
		for (decltype(staticMeshDataManager.GetCount()) i = 0;
			i < staticMeshDataManager.GetCount();
			i++)
		{
			StaticMeshComponent& staticMesh = staticMeshDataManager[i];
			GameObject* owner = staticMeshDataManager.GetOwnerByIndex(i);
			TransformComponent* transform = transformDataManager.GetComponentPointer(owner->GetInnerComponentHandle<TransformComponent>());
			//Setting per Mesh Data
			glUniformMatrix4fv(5,1,GL_FALSE,glm::value_ptr(transform->GetModelMatrix()));
			glDrawArrays(GL_TRIANGLES, 0, 36);
		}
		
		
		RenderImGui(eventManager);
		
	}
}