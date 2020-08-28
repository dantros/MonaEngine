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
			"layout(location = 4) uniform vec3 cameraPosition;\n"
			"layout(location = 5) uniform mat4 worldMatrix;\n"
			"void main()\n"
			"{\n"
			"	 vec4 vertices[6] = vec4[6](vec4(0.25, -0.25, 0.5, 1.0), vec4(-0.25, -0.25, 0.5, 1.0), vec4(0.25, 0.25, 0.5, 1.0), vec4(-0.25, 0.25, 0.5, 1.0), vec4(-0.25, -0.25, 0.5, 1.0), vec4(0.25, 0.25, 0.5, 1.0)); \n"
			"    gl_Position = worldMatrix * (vec4(0.2,0.2,0.2,1.0)*vertices[gl_VertexID]);\n"
			"}\n";

		const char* fragment_shader_text =
			"#version 450 core \n"
			"out vec4 color;\n"
			"void main()\n"
			"{\n"
			"    color = vec4(0.3, 0.3, 0.8, 1.0);\n"
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
				ComponentManager<StaticMeshComponent>& staticMeshDataManager,
				ComponentManager<TransformComponent>& transformDataManager,
				ComponentManager<CameraComponent>& cameraDataManager) noexcept 
	{
		glClear(GL_COLOR_BUFFER_BIT);
		//Setting Scene Data

		//TODO(BYRON) : This should be replace with getting cameraComponentfrom code.
		CameraComponent& camera = cameraDataManager[0];
		GameObject* cameraOwner = cameraDataManager.GetOwnerByIndex(0);
		TransformComponent* cameraTransform = transformDataManager.GetComponentPointer(cameraOwner->GetInnerComponentHandle<TransformComponent>());
		glm::vec3 cameraPosition = cameraTransform->GetLocalTranslation();
		glUniform3fv(4, 1, glm::value_ptr(cameraPosition));

		//Iterating over each static mesh
		for (decltype(staticMeshDataManager.GetCount()) i = 0;
			i < staticMeshDataManager.GetCount();
			i++)
		{
			StaticMeshComponent& staticMesh = staticMeshDataManager[i];
			GameObject* owner = staticMeshDataManager.GetOwnerByIndex(i);
			TransformComponent* transform = transformDataManager.GetComponentPointer(owner->GetInnerComponentHandle<TransformComponent>());
			//Setting per Mesh Data
			
			//MONA_LOG_INFO("Rendering Mesh {0}, at position ({1},{2},{3})", i, position.x, position.y, position.z);
			glUniformMatrix4fv(5,1,GL_FALSE,glm::value_ptr(transform->GetModelMatrix()));
			glDrawArrays(GL_TRIANGLES, 0, 6);
		}
		
		
		RenderImGui(eventManager);
		
	}
}