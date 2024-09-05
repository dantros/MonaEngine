#include "IKNavigationDebugDraw.hpp"
#include <assert.h>
#include "../Core/Log.hpp"
#include <glm/gtc/type_ptr.hpp>
#include <glad/glad.h>
namespace Mona {

	void IKNavigationDebugDraw::StartUp() noexcept {
		dd::initialize(this);
		glCreateVertexArrays(1, &VAO);
		glLineWidth(lineWidth);
	}
	void IKNavigationDebugDraw::ShutDown() noexcept {
		dd::shutdown();
	}


	void IKNavigationDebugDraw::drawLineList(const dd::DrawVertex* lines, int count, bool depthEnabled)	{
		assert(lines != nullptr);
		assert(count > 0 && count <= DEBUG_DRAW_VERTEX_BUFFER_SIZE);

		for (int i = 0; i < count-1; i++) {
			glBindVertexArray(VAO);
			glUniform3f(2, lines[i].line.x, lines[i].line.y, lines[i].line.z);
			glUniform3f(3, lines[i+1].line.x, lines[i+1].line.y, lines[i+1].line.z);
			glUniform3f(5, lines[i].line.r, lines[i].line.g, lines[i].line.b);
			glDrawArrays(GL_LINES, 0, 2);
		}		
	}

}