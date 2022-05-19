#include "IKRig.hpp"
#include "../Core/Log.hpp"
#include "../Utilities/FuncUtils.hpp"


namespace Mona {

	IKNode::IKNode(std::string jointName, int jointIndex, IKNode* parent, float weight) {
		m_jointName = jointName;
		m_jointIndex = jointIndex;
		m_parent = parent;
		m_weight = weight;
	}

	IKRig::IKRig(std::vector<BVHData*> bvhAnims, RigData rigData, bool adjustFeet, AnimationController* animationController) {
		m_animationController = animationController;
		m_adjustFeet = adjustFeet;
		m_bvhAnims = bvhAnims;

		if (bvhAnims.size() == 0) {
			MONA_LOG_ERROR("Must include at least one bvh animation.");
		}

		BVHData* staticData = m_bvhAnims[0];
		std::vector<int> topology = staticData->getTopology();
		std::vector<std::string> jointNames = staticData->getJointNames();
		MONA_ASSERT(topology.size() == jointNames.size(), "Topology and jointNames arrays should have the same size");
		// construimos el ikRig
		m_nodes = std::vector<IKNode>(jointNames.size());
		m_nodes[0] = IKNode(jointNames[0], 0);
		for (int i = 1; i < jointNames.size(); i++) {
			m_nodes[i] = IKNode(jointNames[i], i, &m_nodes[topology[i]]);
		}

		
		std::vector<ChainEnds> dataArr = { rigData.spine, rigData.leftLeg, rigData.rightLeg, rigData.leftArm, rigData.rightArm, rigData.leftFoot, rigData.rightFoot };
		std::vector<std::pair<int, int>*> nodeTargets = { &m_spine, &m_leftLeg, &m_rightLeg, &m_leftArm, &m_rightArm, &m_leftFoot, &m_rightFoot };
		for (int i = 0; i < dataArr.size(); i++) { // construccion de las cadenas principales
			int eeIndex = funcUtils::findIndex(staticData->getJointNames(), dataArr[i].endEffectorName);
			if (eeIndex != -1) {
				int chainStartIndex = -1;
				IKNode* currentNode = &m_nodes[eeIndex];
				currentNode = currentNode->m_parent;
				while (currentNode != nullptr) {
					if (currentNode->m_jointName == dataArr[i].startJointName) {
						chainStartIndex = currentNode->m_jointIndex;
						break; 
					}
					currentNode = currentNode->m_parent;
				}
				if (chainStartIndex != -1) {
					(*nodeTargets[i]).first = chainStartIndex;
					(*nodeTargets[i]).second = eeIndex;
				}else { 
					MONA_LOG_ERROR("Starting joint and end effector were not on the same chain!"); 
				}
			}
		}

		// setear constraints y pesos
		for (int i = 0; i < m_nodes.size(); i++) {
			JointData currData = rigData.jointData[m_nodes[i].m_jointName];
			if (currData.dataValid) {
				m_nodes[i].m_freeAxis = currData.freeAxis;
				m_nodes[i].m_minAngle = currData.minAngle;
				m_nodes[i].m_maxAngle = currData.maxAngle;
				m_nodes[i].m_weight = currData.weight;
			}
		}
	}


	void RigData::setJointData(std::string jointName, float minAngle, float maxAngle, bool freeAxis = false, float weight = 1) {
		if (jointName == "") {
			MONA_LOG_ERROR("jointName cannot be empty string.");
			return;
		}
		if (minAngle <= maxAngle) {
			MONA_LOG_ERROR("maxAngle must be equal or greater than minAngle.");
			return;
		}
		jointData[jointName].minAngle = minAngle;
		jointData[jointName].maxAngle = maxAngle;
		jointData[jointName].freeAxis = freeAxis;
		jointData[jointName].weight = weight;
		jointData[jointName].dataValid = true;
	}

}
