#include "Kinematics.hpp"


namespace Mona {

	GradientDescentIK::GradientDescentIK(IKRig* ikRig) {
		m_ikRig = ikRig;
	}

	ForwardKinematics::ForwardKinematics(IKRig* ikRig) {
		m_ikRig = ikRig;
	}

	std::vector<glm::vec3> ForwardKinematics::ModelSpacePositions(AnimationIndex animIndex) {
		std::vector<glm::vec3> modelSpacePos(m_ikRig->m_nodes.size());
		auto anim = m_ikRig->m_animations[animIndex];
		auto& config = m_ikRig->m_animationConfigs[animIndex];
		modelSpacePos[0] = { 0,0,0 };
		// root
		modelSpacePos[0] = modelSpacePos[0] * glm::toMat3(config.getBaseJointRotations()[0].getQuatRotation()) + config.getJointPositions()[0];
		for (int i = 1; i < m_ikRig->m_nodes.size(); i++) {
			modelSpacePos[i] = modelSpacePos[m_ikRig->GetTopology()[i]] * glm::toMat3(config.getBaseJointRotations()[i].getQuatRotation())
				+ config.getJointPositions()[i];
		}
		return modelSpacePos;
	}

}