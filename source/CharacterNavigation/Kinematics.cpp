#include "Kinematics.hpp"
#include "../Core/GlmUtils.hpp"


namespace Mona {

	ForwardKinematics::ForwardKinematics(IKRig* ikRig) {
		m_ikRig = ikRig;
	}

	GradientDescentIK::GradientDescentIK(IKRig* ikRig) {
		m_ikRig = ikRig;
		m_forwardKinematics = ForwardKinematics(ikRig);
	}

	

	std::vector<glm::vec3> ForwardKinematics::ModelSpacePositions(AnimationIndex animIndex, bool useDynamicRotations) {
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

	std::vector<glm::mat4x4> ForwardKinematics::ModelSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations) {
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

	std::vector<glm::mat4x4> ForwardKinematics::JointSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations) {
		std::vector<glm::mat4x4> jointSpaceTr(m_ikRig->GetTopology().size());
		auto anim = m_ikRig->GetAnimation(animIndex);
		auto& config = m_ikRig->GetAnimationConfig(animIndex);
		auto& rotations = useDynamicRotations ? config.getDynamicJointRotations() : config.getBaseJointRotations();
		for (int i = 0; i < m_ikRig->GetTopology().size(); i++) {
			jointSpaceTr[i] = glmUtils::scaleToMat4(config.getJointScales()[i]) *glmUtils::rotationToMat4(rotations[i].getQuatRotation()) * 
				glmUtils::translationToMat4(config.getJointPositions()[i]);
		}
		return jointSpaceTr;
	}

}