#include "IKNavigationDebugDraw.hpp"
#include <assert.h>
#include "../Core/Log.hpp"
namespace Mona {

	void IKNavigationDebugDraw::StartUp() noexcept {
		dd::initialize(this);
	}
	void IKNavigationDebugDraw::ShutDown() noexcept {
		dd::shutdown();
	}
	static void checkGLError(const char* file, const int line)
	{
		GLenum err;
		while ((err = glGetError()) != GL_NO_ERROR)
		{
			MONA_LOG_ERROR("IKNavigationDebugDraw: GL_ERROR");
		}
	}
	void IKNavigationDebugDraw::drawPointList(const dd::DrawVertex* points, int count, bool depthEnabled)
	{
		assert(points != nullptr);
		assert(count > 0 && count <= DEBUG_DRAW_VERTEX_BUFFER_SIZE);

		glBindVertexArray(linePointVAO);
		glUseProgram(linePointProgram);

		glUniformMatrix4fv(linePointProgram_MvpMatrixLocation,
			1, GL_FALSE, toFloatPtr(mvpMatrix));

		if (depthEnabled)
		{
			glEnable(GL_DEPTH_TEST);
		}
		else
		{
			glDisable(GL_DEPTH_TEST);
		}

		// NOTE: Could also use glBufferData to take advantage of the buffer orphaning trick...
		glBindBuffer(GL_ARRAY_BUFFER, linePointVBO);
		glBufferSubData(GL_ARRAY_BUFFER, 0, count * sizeof(dd::DrawVertex), points);

		// Issue the draw call:
		glDrawArrays(GL_POINTS, 0, count);

		glUseProgram(0);
		glBindVertexArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		checkGLError(__FILE__, __LINE__);
	}

	void IKNavigationDebugDraw::drawLineList(const dd::DrawVertex* lines, int count, bool depthEnabled)
	{
		assert(lines != nullptr);
		assert(count > 0 && count <= DEBUG_DRAW_VERTEX_BUFFER_SIZE);

		glBindVertexArray(linePointVAO);
		glUseProgram(linePointProgram);

		glUniformMatrix4fv(linePointProgram_MvpMatrixLocation,
			1, GL_FALSE, toFloatPtr(mvpMatrix));

		if (depthEnabled)
		{
			glEnable(GL_DEPTH_TEST);
		}
		else
		{
			glDisable(GL_DEPTH_TEST);
		}

		// NOTE: Could also use glBufferData to take advantage of the buffer orphaning trick...
		glBindBuffer(GL_ARRAY_BUFFER, linePointVBO);
		glBufferSubData(GL_ARRAY_BUFFER, 0, count * sizeof(dd::DrawVertex), lines);

		// Issue the draw call:
		glDrawArrays(GL_LINES, 0, count);

		glUseProgram(0);
		glBindVertexArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		checkGLError(__FILE__, __LINE__);
	}

	

}