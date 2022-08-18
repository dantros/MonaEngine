#pragma once
#ifndef IKNAVIGATIONDEBUGDRAW_HPP
#define IKNAVIGATIONDEBUGDRAW_HPP
#include <debug_draw.hpp>
#include <glm/glm.hpp>
#include "../Rendering/ShaderProgram.hpp"
namespace Mona {
	class IKNavigationDebugDraw : public dd::RenderInterface {
	public:
		IKNavigationDebugDraw() = default;
		~IKNavigationDebugDraw() = default;
		void StartUp() noexcept;
		void ShutDown() noexcept;
		void drawLineList(const dd::DrawVertex* lines, int count, bool depthEnabled) override;
		bool m_drawEETargetCurves = true;
		bool m_drawHipTargetCurve = true;
		glm::vec3 m_eeCurveColor = glm::vec3(1.0f, 0.0f, 0.0f);
		glm::vec3 m_hipCurveColor = glm::vec3(0.0f, 1.0f, 0.0f);
	private:
		uint32_t VAO = 0;
		uint32_t lineWidth = 4;
	};
}
#endif