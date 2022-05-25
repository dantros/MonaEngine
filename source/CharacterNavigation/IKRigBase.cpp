#include "IKRigBase.hpp"

namespace Mona{

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
	JointRotation::JointRotation(float rotationAngle, Vector3f rotationAxis) {
		setRotation(rotationAngle, rotationAxis);
	}
	JointRotation::JointRotation(Quaternion quatRotation) {
		setRotation(quatRotation);
	}
	void JointRotation::setRotation(Quaternion rotation) {
		m_quatRotation = rotation;
	}
	void JointRotation::setRotation(float rotationAngle, Vector3f rotationAxis) {
		m_quatRotation = glm::angleAxis(rotationAngle, rotationAxis);
	}

	IKRigConfigValidator::IKRigConfigValidator(std::vector<IKNode>* nodesPtr, std::vector<int>* topologyPtr) {
		m_nodes = nodesPtr;
		m_topology = topologyPtr;
	}
    
}