#pragma once
#ifndef IKNAVIGATIONDEBUGDRAW_HPP
#define IKNAVIGATIONDEBUGDRAW_HPP
#include <debug_draw.hpp>
#include <glm/glm.hpp>
namespace Mona {
	class IKNavigationDebugDraw : public dd::RenderInterface {
	public:
		IKNavigationDebugDraw();
		~IKNavigationDebugDraw();
		void StartUp() noexcept;
		void ShutDown() noexcept;
		void drawLineList(const dd::DrawVertex* lines, int count, bool depthEnabled) override;
		bool m_drawEETargetCurves = false;
		bool m_drawHipTargetCurve = false;
		glm::vec3 m_eeCurveColor = glm::vec3(1.0f, 0.0f, 0.0f);
		glm::vec3 m_hipCurveColor = glm::vec3(0.0f, 1.0f, 0.0f);
		// The "model-view-projection" matrix for the scene.
		glm::mat4 mvpMatrix;
	private:
		void setupShaderPrograms();
		void setupVertexBuffers();
		static void compileShader(const uint32_t shader);
		static void linkProgram(const uint32_t program);
		uint32_t linePointProgram;
		int32_t  linePointProgram_MvpMatrixLocation;

		uint32_t linePointVAO;
		uint32_t linePointVBO;

		static const char* linePointVertShaderSrc;
		static const char* linePointFragShaderSrc;
	};
}
#endif