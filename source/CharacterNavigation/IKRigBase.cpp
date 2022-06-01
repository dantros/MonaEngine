#include "IKRigBase.hpp"

namespace Mona{

	IKRigConfig::IKRigConfig(std::shared_ptr<AnimationClip> animation, AnimIndex animationIndex) {
		jointPositions.reserve(animation->m_animationTracks.size());
		jointScales.reserve(animation->m_animationTracks.size());
		timeStamps.reserve(animation->m_animationTracks.size()* animation->m_animationTracks[0].rotationTimeStamps.size());
		baseJointRotations.resize(animation->m_animationTracks.size());
		dynamicJointRotations.resize(animation->m_animationTracks.size());
		for (int i = 0; i < animation->m_animationTracks.size();i++) {
			jointPositions.push_back(animation->m_animationTracks[i].positions[0]);
			jointScales.push_back(animation->m_animationTracks[i].scales[0]);
			for (int j = 0; j < animation->m_animationTracks[i].rotationTimeStamps.size(); j++) {
				timeStamps.push_back(animation->m_animationTracks[i].rotationTimeStamps[j]);
			}
		}
		// ordenamos las timestamps y eliminamos las repetidas
		std::sort(timeStamps.begin(), timeStamps.end());
		int index = 0;
		while (index < timeStamps.size()-1) {
			if (timeStamps[index] == timeStamps[index + 1]) {
				timeStamps.erase(timeStamps.begin() + index + 1);
			}
			else {
				index += 1;
			}
		}
		animIndex = animationIndex;

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
		if (leftLeg.startJointName.empty() || leftLeg.endEffectorName.empty() || rightLeg.startJointName.empty() || rightLeg.endEffectorName.empty()) {
			MONA_LOG_ERROR("RigData: Legs cannot be empty");
			return false;
		}
		return true;
	}
	JointRotation::JointRotation() {
		setRotation({ 0,0,0,1 });
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

	IKRigConfigValidator::IKRigConfigValidator(std::vector<IKNode>* nodesPtr, std::vector<int>* topologyPtr) {
		m_nodes = nodesPtr;
		m_topology = topologyPtr;
	}
    
}