#include "Kinematics.hpp"
#include "../Core/GlmUtils.hpp"


namespace Mona {

	Kinematics::Kinematics(IKRig* ikRig) {
		m_ikRig = ikRig;
	}

	GradientDescentIK::GradientDescentIK(IKRig* ikRig) {
	}

	

	std::vector<glm::vec3> Kinematics::ModelSpacePositions(AnimationIndex animIndex, bool useDynamicRotations) {
		std::vector<glm::vec3> modelSpacePos(m_ikRig->GetTopology().size());
		auto anim = m_ikRig->GetAnimation(animIndex);
		auto& config = m_ikRig->GetAnimationConfig(animIndex);
		auto& rotations = useDynamicRotations ? config.getDynamicJointRotations() : config.getBaseJointRotations();
		modelSpacePos[0] = { 0,0,0 };
		// root
		modelSpacePos[0] = modelSpacePos[0] * glmUtils::scaleToMat3(config.getJointScales()[0])*
			glmUtils::rotationToMat3(config.getBaseJointRotations()[0].getQuatRotation()) + config.getJointPositions()[0];
		for (int i = 1; i < m_ikRig->GetTopology().size(); i++) {
			modelSpacePos[i] = modelSpacePos[m_ikRig->GetTopology()[i]] * glmUtils::scaleToMat3(config.getJointScales()[i]) *
				glmUtils::rotationToMat3(config.getBaseJointRotations()[i].getQuatRotation()) + config.getJointPositions()[i];
		}
		return modelSpacePos;
	}

	glm::vec3 Kinematics::ModelSpaceJointPosition(AnimationIndex animIndex, JointIndex jointIndex, bool useDynamicRotations) {
		if (!(jointIndex < m_ikRig->GetTopology().size())) {
			MONA_LOG_ERROR("Kinematics: JointIndex {0} is out of bounds", jointIndex);
			return glm::vec3(0);
		}
		std::vector<glm::vec3> modelSpacePos(m_ikRig->GetTopology().size());
		auto anim = m_ikRig->GetAnimation(animIndex);
		auto& config = m_ikRig->GetAnimationConfig(animIndex);
		auto& rotations = useDynamicRotations ? config.getDynamicJointRotations() : config.getBaseJointRotations();
		modelSpacePos[0] = { 0,0,0 };
		// root
		modelSpacePos[0] = modelSpacePos[0] * glmUtils::scaleToMat3(config.getJointScales()[0]) *
			glmUtils::rotationToMat3(config.getBaseJointRotations()[0].getQuatRotation()) + config.getJointPositions()[0];
		for (int i = 1; i <= jointIndex; i++) {
			modelSpacePos[i] = modelSpacePos[m_ikRig->GetTopology()[i]] * glmUtils::scaleToMat3(config.getJointScales()[i]) *
				glmUtils::rotationToMat3(config.getBaseJointRotations()[i].getQuatRotation()) + config.getJointPositions()[i];
		}
		return modelSpacePos[jointIndex];

	}
	std::vector<glm::mat4x4> Kinematics::ModelSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations) {
		std::vector<glm::mat4x4> modelSpaceTr(m_ikRig->GetTopology().size());
		auto anim = m_ikRig->GetAnimation(animIndex);
		auto& config = m_ikRig->GetAnimationConfig(animIndex);
		auto& rotations = useDynamicRotations ? config.getDynamicJointRotations() : config.getBaseJointRotations();
		// root
		modelSpaceTr[0] = glmUtils::scaleToMat4(config.getJointScales()[0])*
			glmUtils::rotationToMat4(rotations[0].getQuatRotation())*glmUtils::translationToMat4(config.getJointPositions()[0]);
		for (int i = 1; i < m_ikRig->GetTopology().size(); i++) {
			modelSpaceTr[i] = modelSpaceTr[m_ikRig->GetTopology()[i]]* glmUtils::scaleToMat4(config.getJointScales()[i]) *
				glmUtils::rotationToMat4(rotations[i].getQuatRotation()) * glmUtils::translationToMat4(config.getJointPositions()[i]);
		}
		return modelSpaceTr;
	}
	glm::mat4x4 Kinematics::ModelSpaceJointTransform(AnimationIndex animIndex, JointIndex jointIndex, bool useDynamicRotations) {
		if (!(jointIndex < m_ikRig->GetTopology().size())) {
			MONA_LOG_ERROR("Kinematics: JointIndex {0} is out of bounds", jointIndex);
			return glm::mat4x4(0);
		}
		std::vector<glm::mat4x4> modelSpaceTr(m_ikRig->GetTopology().size());
		auto anim = m_ikRig->GetAnimation(animIndex);
		auto& config = m_ikRig->GetAnimationConfig(animIndex);
		auto& rotations = useDynamicRotations ? config.getDynamicJointRotations() : config.getBaseJointRotations();
		// root
		modelSpaceTr[0] = glmUtils::scaleToMat4(config.getJointScales()[0]) *
			glmUtils::rotationToMat4(rotations[0].getQuatRotation()) * glmUtils::translationToMat4(config.getJointPositions()[0]);
		for (int i = 1; i < jointIndex; i++) {
			modelSpaceTr[i] = modelSpaceTr[m_ikRig->GetTopology()[i]] * glmUtils::scaleToMat4(config.getJointScales()[i]) *
				glmUtils::rotationToMat4(rotations[i].getQuatRotation()) * glmUtils::translationToMat4(config.getJointPositions()[i]);
		}
		return modelSpaceTr[jointIndex];
	}

}