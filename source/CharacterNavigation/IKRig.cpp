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
		for (int i = 0; i < jointNames.size(); i++) {
			m_nodes[i] = IKNode(jointNames[i], i);
			if (i > 0) {
				m_nodes[i].m_parent = &m_nodes[topology[i]];
			}
		}
		for (int i = 1; i < topology.size(); i++) { // guardar los hijos
			m_nodes[topology[i]].m_children.push_back(&m_nodes[i]);
		}

		
		std::vector<ChainData> dataArr = { rigData.spine, rigData.leftLeg, rigData.rightLeg, rigData.leftArm, rigData.rightArm, rigData.leftFoot, rigData.rightFoot };
		std::vector<std::vector<IKNode*>*> nodeTargets = { &m_spineChain, &m_leftLegChain, &m_rightLegChain, &m_leftArmChain, &m_rightArmChain, &m_leftFootChain, &m_rightFootChain };
		for (int i = 0; i < dataArr.size(); i++) { // construccion de las cadenas principales
			int eeIndex = funcUtils::findIndex(staticData->getJointNames(), dataArr[i].endEffectorName);
			if (eeIndex != -1) {
				IKNode* currentNode = &m_nodes[eeIndex];
				(*nodeTargets[i]).push_back(currentNode);
				while (currentNode->m_parent != nullptr)
				{
					currentNode = currentNode->m_parent;
					(*nodeTargets[i]).push_back(currentNode);
					if (currentNode->m_jointName == dataArr[i].startJointName) { break; }
					if (currentNode->m_parent == nullptr) { MONA_LOG_ERROR("Starting joint and end effector were not on the same chain!"); }
				}
				
			}
		}

		if (adjustFeet) {

		}
	}




}
