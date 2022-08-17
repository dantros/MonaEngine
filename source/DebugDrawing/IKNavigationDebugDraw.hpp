#pragma once
#ifndef IKNAVIGATIONDEBUGDRAW_HPP
#define IKNAVIGATIONDEBUGDRAW_HPP
#include <debug_draw.hpp>
#include <vectormath/vectormath.h>
#include <glad/glad.h>
namespace Mona {
	class IKNavigationDebugDraw : public dd::RenderInterface {
	public:
		void StartUp() noexcept;
		void ShutDown() noexcept;
		void drawPointList(const dd::DrawVertex* points, int count, bool depthEnabled) override;
		void drawLineList(const dd::DrawVertex* lines, int count, bool depthEnabled) override;
		bool m_drawEETargetCurves = false;
		bool m_drawHipTargetCurve = false;
		// The "model-view-projection" matrix for the scene.
		Matrix4 mvpMatrix;
	private:

		GLuint linePointProgram;
		GLint  linePointProgram_MvpMatrixLocation;

		GLuint textProgram;
		GLint  textProgram_GlyphTextureLocation;
		GLint  textProgram_ScreenDimensions;

		GLuint linePointVAO;
		GLuint linePointVBO;

		GLuint textVAO;
		GLuint textVBO;

		static const char* linePointVertShaderSrc;
		static const char* linePointFragShaderSrc;

		static const char* textVertShaderSrc;
		static const char* textFragShaderSrc;

	};
}
#endif