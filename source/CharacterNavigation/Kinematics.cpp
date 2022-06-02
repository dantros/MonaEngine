#include "Kinematics.hpp"


namespace Mona {

	Kinematics::Kinematics(IKRig* ikRig) {
		m_ikRig = ikRig;
	}

	GradientDescentIK::GradientDescentIK(IKRig* ikRig) {
	}

	

	std::vector<glm::vec3> Kinematics::ModelSpacePositions(AnimationIndex animIndex) {
		std::vector<glm::vec3> modelSpacePos(m_ikRig->GetTopology().size());
		auto anim = m_ikRig->GetAnimation(animIndex);
		auto& config = m_ikRig->GetAnimationConfig(animIndex);
		modelSpacePos[0] = { 0,0,0 };
		// root
		modelSpacePos[0] = modelSpacePos[0] * glm::toMat3(config.getBaseJointRotations()[0].getQuatRotation()) + config.getJointPositions()[0];
		for (int i = 1; i < m_ikRig->GetTopology().size(); i++) {
			modelSpacePos[i] = modelSpacePos[m_ikRig->GetTopology()[i]] * glm::toMat3(config.getBaseJointRotations()[i].getQuatRotation())
				+ config.getJointPositions()[i];
		}
		return modelSpacePos;
	}

}