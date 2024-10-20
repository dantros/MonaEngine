#include "DebugDrawingSystem.hpp"



#ifndef NDEBUG
#include "../Core/Log.hpp"
#include <imgui.h>
#include "examples/imgui_impl_glfw.h"
#include "examples/imgui_impl_opengl3.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/type_ptr.hpp>
#include "../PhysicsCollision/PhysicsCollisionSystem.hpp"
#include "../Core/Config.hpp"
void GLAPIENTRY MessageCallback(GLenum source,
	GLenum type,
	GLuint id,
	GLenum severity,
	GLsizei length,
	const GLchar* message,
	const void* userParam)
{
	MONA_LOG_ERROR("OpenGL Error: type = {0}, message = {1}", type, message);

}

namespace Mona {


	void DebugDrawingSystem_physics::Draw(EventManager& eventManager, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) noexcept {

		glUseProgram(m_lineShader.GetProgramID());
		glUniformMatrix4fv(0, 1, GL_FALSE, glm::value_ptr(projectionMatrix));
		glUniformMatrix4fv(1, 1, GL_FALSE, glm::value_ptr(viewMatrix));
		m_physicsWorldPtr->debugDrawWorld();



		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		{
			ImGui::Begin("Debug Settings:");
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::Separator();
			ImGui::Text("Collision Physics System Debug Draw (Bullet):");
			ImGui::Checkbox("Draw Wireframe", &(m_bulletDebugDrawPtr->m_bDrawWireframe));
			ImGui::Checkbox("Draw ContactPoints", &(m_bulletDebugDrawPtr->m_bDrawContactsPoints));
			ImGui::Checkbox("Draw AABB", &(m_bulletDebugDrawPtr->m_bDrawAABB));
			ImGui::End();
		}
		eventManager.Publish(DebugGUIEvent());
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
	}
	void DebugDrawingSystem_physics::StartUp(PhysicsCollisionSystem* physicsSystemPtr) noexcept {
		auto& config = Mona::Config::GetInstance();
		m_lineShader = ShaderProgram(
			config.getPathOfEngineAsset("Shaders/LineVS.vs"),
			config.getPathOfEngineAsset("Shaders/LinePS.ps")
		);
		m_physicsWorldPtr = physicsSystemPtr->GetPhysicsWorldPtr();
		m_bulletDebugDrawPtr.reset(new BulletDebugDraw());
		m_bulletDebugDrawPtr->StartUp();
		m_bulletDebugDrawPtr->setDebugMode(0);
		m_physicsWorldPtr->setDebugDrawer(m_bulletDebugDrawPtr.get());

		glEnable(GL_DEBUG_OUTPUT);
		glDebugMessageCallback(MessageCallback, 0);
		glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_NOTIFICATION, 0, nullptr, GL_FALSE);

		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImGuiIO& io = ImGui::GetIO(); (void)io;
		ImGui::StyleColorsDark();
		ImGui_ImplGlfw_InitForOpenGL(glfwGetCurrentContext(), true);
		ImGui_ImplOpenGL3_Init("#version 450");
	}

	void DebugDrawingSystem_physics::ShutDown() noexcept {
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext();
	}


	void DebugDrawingSystem_ikNav::StartUp(IKNavigationSystem* ikNavSystemPtr)  noexcept {
		auto& config = Mona::Config::GetInstance();
		m_lineShader = ShaderProgram(
			config.getPathOfEngineAsset("Shaders/LineVS.vs"),
			config.getPathOfEngineAsset("Shaders/LinePS.ps")
		);
		m_ikNavSystemPtr = ikNavSystemPtr;
		m_ikNavDebugDrawPtr.reset(new IKNavigationDebugDraw);
		m_ikNavDebugDrawPtr->StartUp();

		glEnable(GL_DEBUG_OUTPUT);
		glDebugMessageCallback(MessageCallback, 0);
		glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_NOTIFICATION, 0, nullptr, GL_FALSE);

		IMGUI_CHECKVERSION();
		ImGui::CreateContext();
		ImGuiIO& io = ImGui::GetIO(); (void)io;
		ImGui::StyleColorsDark();
		ImGui_ImplGlfw_InitForOpenGL(glfwGetCurrentContext(), true);
		ImGui_ImplOpenGL3_Init("#version 450");
	
	}
	void DebugDrawingSystem_ikNav::Draw(EventManager& eventManager, const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) noexcept {
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
		{
			ImGui::Begin("Debug Settings:");
			ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
			ImGui::Separator();
			ImGui::Text("IK Navigation System Debug Draw:");
			ImGui::Checkbox("Draw EndEffector Target Curves", &(m_ikNavDebugDrawPtr->m_drawEETargetCurves));
			ImGui::Checkbox("Draw EndEffector Real Curves", &(m_ikNavDebugDrawPtr->m_drawEERealCurves));
			ImGui::Checkbox("Draw Hip Target Curves", &(m_ikNavDebugDrawPtr->m_drawHipTargetCurve));
			ImGui::End();
		}
		eventManager.Publish(DebugGUIEvent());
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
		
		
		glUseProgram(m_lineShader.GetProgramID());
		glUniformMatrix4fv(0, 1, GL_FALSE, glm::value_ptr(projectionMatrix));
		glUniformMatrix4fv(1, 1, GL_FALSE, glm::value_ptr(viewMatrix));
		
		std::vector<IKRigController*> ikRigControllers = m_ikNavSystemPtr->getControllersDebug();
		for (int i = 0; i < ikRigControllers.size(); i++) {
			IKRigController* currController = ikRigControllers[i];
			std::vector<IKAnimation>& currentIKAnims = currController->m_ikRig.m_ikAnimations;
			for (int j = 0; j < currentIKAnims.size(); j++) {
				IKAnimation& currentIKAnim = currentIKAnims[j];
				if ((m_ikNavDebugDrawPtr->m_drawEETargetCurves || m_ikNavDebugDrawPtr->m_drawEERealCurves) && currentIKAnim.isActive()) {
					glm::vec3 colorT = m_ikNavDebugDrawPtr->m_eeTargetCurveColor;
					glm::vec3 colorR = m_ikNavDebugDrawPtr->m_eeRealCurveColor;
					for (int k = 0; k < currentIKAnim.m_eeTrajectoryData.size(); k++) {
						LIC<3>& currTargetCurve = currentIKAnim.m_eeTrajectoryData[k].getTargetTrajectory().getEECurve();
						LIC<3>const& currRealCurve = currentIKAnim.m_eeTrajectoryData[k].getSavedPositions();
						int pointNum = currTargetCurve.getNumberOfPoints();
						if (0 < pointNum) {
							std::vector<dd::DrawVertex> linesT(pointNum);
							std::vector<dd::DrawVertex> linesR;
							for (int l = 0; l < pointNum; l++) {
								// posiciones objetivo
								dd::DrawVertex vT;
								glm::vec3 pointT = currTargetCurve.getCurvePoint(l);
								vT.line.r = colorT[0]; vT.line.g = colorT[1]; vT.line.b = colorT[2];
								vT.line.x = pointT[0]; vT.line.y = pointT[1]; vT.line.z = pointT[2];
								linesT[l] = vT;

								// posiciones reales
								if (currRealCurve.inTRange(currTargetCurve.getTValue(l))) {
									dd::DrawVertex vR;
									glm::vec3 pointR = currRealCurve.evalCurve(currTargetCurve.getTValue(l));
									vR.line.r = colorR[0]; vR.line.g = colorR[1]; vR.line.b = colorR[2];
									vR.line.x = pointR[0]; vR.line.y = pointR[1]; vR.line.z = pointR[2];
									linesR.push_back(vR);
								}							

							}
							if (m_ikNavDebugDrawPtr->m_drawEETargetCurves) {
								m_ikNavDebugDrawPtr->drawLineList(&linesT[0], linesT.size(), true);
							}
							if (m_ikNavDebugDrawPtr->m_drawEERealCurves && 0 < linesR.size()) {
								m_ikNavDebugDrawPtr->drawLineList(&linesR[0], linesR.size(), true);
							}
							
						}			
					}
				}
				if (m_ikNavDebugDrawPtr->m_drawHipTargetCurve && currentIKAnim.isActive()) {
					glm::vec3 color = m_ikNavDebugDrawPtr->m_hipCurveColor;
					LIC<3>& hipTargetCurve = currentIKAnim.getHipTrajectoryData()->m_targetPositions;
					int pointNum = hipTargetCurve.getNumberOfPoints();
					if (0 < pointNum) {
						std::vector<dd::DrawVertex> lines(pointNum);
						for (int l = 0; l < pointNum; l++) {
							dd::DrawVertex v;
							glm::vec3 point = hipTargetCurve.getCurvePoint(l);
							v.line.r = color[0]; v.line.g = color[1]; v.line.b = color[2];
							v.line.x = point[0]; v.line.y = point[1]; v.line.z = point[2];
							lines[l] = v;
						}
						m_ikNavDebugDrawPtr->drawLineList(&lines[0], lines.size(), true);
					}					
				}
			}
		}
		dd::flush(0);
	}


	void DebugDrawingSystem_ikNav::ShutDown() noexcept {
		ImGui_ImplOpenGL3_Shutdown();
		ImGui_ImplGlfw_Shutdown();
		ImGui::DestroyContext();
		glDeleteProgram(m_lineShader.GetProgramID());
		m_ikNavDebugDrawPtr->ShutDown();	
		m_ikNavDebugDrawPtr.reset();
	}
}

#endif
