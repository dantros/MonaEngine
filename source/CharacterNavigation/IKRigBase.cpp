#include "IKRigBase.hpp"

namespace Mona{

	IKRigConfig::IKRigConfig(std::shared_ptr<AnimationClip> animation, AnimationIndex animationIndex, ForwardKinematics* forwardKinematics) {
		int frameNum = animation->m_animationTracks[0].rotationTimeStamps.size();
		int jointNum = animation->m_animationTracks.size();
		m_jointPositions.reserve(jointNum);
		m_jointScales.reserve(jointNum);
		m_timeStamps = animation->m_animationTracks[0].rotationTimeStamps;
		m_baseJointRotations.reserve(frameNum);
		m_dynamicJointRotations.resize(jointNum);
		for (int i = 0; i < jointNum;i++) {
			m_jointPositions.push_back(animation->m_animationTracks[i].positions[0]);
			m_jointScales.push_back(animation->m_animationTracks[i].scales[0]);
		}
		for (int i = 0; i < frameNum; i++) {
			m_baseJointRotations.push_back(std::vector<JointRotation>(jointNum));
			for (int j = 0; j < jointNum; j++) {
				m_baseJointRotations[i][j] = JointRotation(animation->m_animationTracks[j].rotations[i]);
			}
		}
		m_animIndex = animationIndex;
		m_forwardKinematics = forwardKinematics;
	}

	std::vector<glm::vec3> IKRigConfig::getModelSpacePositions(bool useDynamicRotations) {
		return m_forwardKinematics->ModelSpacePositions(m_animIndex, useDynamicRotations);
	}
	glm::vec3 IKRigConfig::getModelSpacePosition(JointIndex jointIndex, bool useDynamicRotations) {
		return m_forwardKinematics->ModelSpacePosition(m_animIndex, jointIndex, useDynamicRotations);
	}
	std::vector<glm::mat4> IKRigConfig::getModelSpaceTransforms(bool useDynamicRotations) {
		return m_forwardKinematics->ModelSpaceTransforms(m_animIndex, useDynamicRotations);
	}
	std::vector<glm::mat4> IKRigConfig::getJointSpaceTransforms(bool useDynamicRotations) {
		return m_forwardKinematics->JointSpaceTransforms(m_animIndex, useDynamicRotations);
	}

	IKNode::IKNode(std::string jointName, int jointIndex, IKNode* parent, float weight) {
		m_jointName = jointName;
		m_jointIndex = jointIndex;
		m_parent = parent;
		m_weight = weight;
	}

	void RigData::setJointData(std::string jointName, float minAngle, float maxAngle, float weight, bool enableData) {
		if (jointName == "") {
			MONA_LOG_ERROR("RigData: jointName cannot be empty string.");
			return;
		}
		if (minAngle <= maxAngle) {
			MONA_LOG_ERROR("RigData: maxAngle must be equal or greater than minAngle.");
			return;
		}
		jointData[jointName].minAngle = minAngle;
		jointData[jointName].maxAngle = maxAngle;
		jointData[jointName].weight = weight;
		jointData[jointName].enableIKRotation = enableData;
	}

	JointData RigData::getJointData(std::string jointName) {
		return JointData(jointData[jointName]);
	}
	void RigData::enableJointData(std::string jointName, bool enableData) {
		jointData[jointName].enableIKRotation = enableData;
	}

	bool RigData::isValid() {
		if (leftLeg.baseJointName.empty() || leftLeg.endEffectorName.empty() || rightLeg.baseJointName.empty() || rightLeg.endEffectorName.empty()) {
			MONA_LOG_ERROR("RigData: Legs cannot be empty");
			return false;
		}
		return true;
	}

	JointRotation::JointRotation() {
		setRotation(glm::identity<glm::fquat>());
	}
	JointRotation::JointRotation(float rotationAngle, glm::vec3 rotationAxis) {
		setRotation(rotationAngle, rotationAxis);
	}
	JointRotation::JointRotation(glm::fquat quatRotation) {
		setRotation(quatRotation);
	}
	void JointRotation::setRotation(glm::fquat rotation) {
		m_quatRotation = rotation;
		m_rotationAngle = glm::angle(m_quatRotation);
		m_rotationAxis = glm::axis(m_quatRotation);
	}
	void JointRotation::setRotation(float rotationAngle, glm::vec3 rotationAxis) {
		m_quatRotation = glm::angleAxis(rotationAngle, rotationAxis);
		m_rotationAngle = rotationAngle;
		m_rotationAxis = rotationAxis;
	}

	void JointRotation::setRotationAngle(float rotationAngle) {
		m_quatRotation = glm::angleAxis(rotationAngle, m_rotationAxis);
		m_rotationAngle = rotationAngle;
	}

	void JointRotation::setRotationAxis(glm::vec3 rotationAxis) {
		m_quatRotation = glm::angleAxis(m_rotationAngle, rotationAxis);
		m_rotationAxis = rotationAxis;
	}
    
}