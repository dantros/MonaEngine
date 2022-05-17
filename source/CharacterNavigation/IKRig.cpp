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
		
		std::vector<ChainData> dataArr = { rigData.spine, rigData.leftLeg, rigData.rightLeg, rigData.leftArm, rigData.rightArm, rigData.leftFoot, rigData.rightFoot };
		std::vector<IKNode**> nodeTargets = { &m_spineEE, &m_leftLegEE, &m_rightLegEE, &m_leftArmEE, &m_rightArmEE, &m_leftFootEE, &m_rightFootEE };
		for (int i = 0; i < dataArr.size(); i++) {
			// buscamos el end effector
			int eeIndex = funcUtils::findIndex(staticData->getJointNames(), dataArr[i].endEffectorName);
			if (eeIndex != -1) {
				IKNode currentNode = IKNode(dataArr[i].endEffectorName, i);
				m_nodes.push_back(currentNode);
				(*nodeTargets[i]) = &m_nodes.back();
				int parentIndex = staticData->getTopology()[eeIndex];
				std::string parentName;
				while (parentIndex != -1) {
					parentName = staticData->getJointNames()[parentIndex];
					currentNode = IKNode(parentName, parentIndex);
					m_nodes.push_back(currentNode);
					m_nodes[m_nodes.size() - 2].m_parent = &m_nodes[m_nodes.size() - 1]; // el padre del nodo anterior es el nodo actual

					if (parentName == dataArr[i].startJointName) { break; }
					if (parentIndex == -1) { MONA_LOG_ERROR("start joint and end effector were not on the same chain!"); }
					parentIndex = staticData->getTopology()[parentIndex]; // guardarmos el indice del padre para la siguiente iteracion
				}
			}
		}

		if (adjustFeet) {

		}
	}



}
