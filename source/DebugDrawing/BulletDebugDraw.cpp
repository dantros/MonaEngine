#include "BulletDebugDraw.hpp"
#include <glad/glad.h>
namespace Mona {
	void BulletDebugDraw::setDebugMode(int debugMode) {
		m_bDrawWireframe = (debugMode & btIDebugDraw::DebugDrawModes::DBG_DrawWireframe);
		m_bDrawContactsPoints = (debugMode & BulletDebugDraw::DBG_DrawContactPoints);
		m_bDrawAABB = (debugMode & btIDebugDraw::DebugDrawModes::DBG_DrawAabb);
	}
	int BulletDebugDraw::getDebugMode() const {
		return ((m_bDrawWireframe? ~0u : 0)  & btIDebugDraw::DebugDrawModes::DBG_DrawWireframe ) |
			((m_bDrawContactsPoints? ~0u : 0) & BulletDebugDraw::DBG_DrawContactPoints) |
			((m_bDrawAABB? ~0u : 0) & btIDebugDraw::DebugDrawModes::DBG_DrawAabb);
	}

	void BulletDebugDraw::StartUp() noexcept {
		glCreateVertexArrays(1, &VAO);
		glLineWidth(3);
	}


	void BulletDebugDraw::drawLine(const btVector3& from, const btVector3& to, const btVector3& color) {
		glBindVertexArray(VAO);
		glUniform3f(2, from.x(), from.y(), from.z());
		glUniform3f(3, to.x(), to.y(), to.z());
		glUniform3f(5, color.x(), color.y(), color.z());
		glDrawArrays(GL_LINES, 0, 2);


	}

	void BulletDebugDraw::drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color) {
		btVector3 const startPoint = PointOnB;
		btVector3 const endPoint = PointOnB + normalOnB * distance;
		drawLine(startPoint, endPoint, color);

	}

	

}