#include "IKRig.hpp"
#include "../Core/Log.hpp"
#include "../Core/FuncUtils.hpp"


namespace Mona {

	IKRig::IKRig(std::shared_ptr<Skeleton> skeleton, RigData rigData, InnerComponentHandle skeletalMeshHandle) :
		m_skeletalMeshHandle(skeletalMeshHandle)
	{
		m_skeleton = skeleton;
		m_forwardKinematics = ForwardKinematics(this);
		ChainIndex hipIKChain = 0;
		m_trajectoryGenerator = TrajectoryGenerator(this);

		auto topology = getTopology();
		auto jointNames = getJointNames();

		// construimos el ikRig
		m_nodes = std::vector<IKNode>(jointNames.size());
		m_nodes[0] = IKNode(jointNames[0], 0);
		for (int i = 1; i < jointNames.size(); i++) {
			m_nodes[i] = IKNode(jointNames[i], i, &m_nodes[topology[i]]);
		}

		// seteo de la cadera
		JointIndex hipInd = funcUtils::findIndex(jointNames, rigData.hipJointName);
		MONA_ASSERT(hipInd != -1, "IKRig: Input hip joint name was not a correct joint name.");
		m_hipJoint = hipInd;	
		
		// construccion de las cadenas principales
		m_ikChains = { buildIKChain(rigData.leftLeg, "leftLeg"), buildIKChain(rigData.rightLeg, "rightLeg"),
		buildIKChain(rigData.leftFoot, "leftFoot"), buildIKChain(rigData.rightFoot, "rightFoot") };

		// setear constraints y pesos
		for (int i = 0; i < m_nodes.size(); i++) {
			JointData currData = rigData.jointData[m_nodes[i].m_jointName];
			if (currData.enableIKRotation) {
				m_nodes[i].m_minAngle = currData.minAngle;
				m_nodes[i].m_maxAngle = currData.maxAngle;
				m_nodes[i].m_weight = currData.weight;
			}
		}
		// setear cinematica inversa
		std::vector<ChainIndex> ikChains;
		for (ChainIndex i = 0; i < m_ikChains.size(); i++) {
			ikChains.push_back(i);
		}
		m_inverseKinematics = InverseKinematics(this, ikChains);

		// setear el la altura del rig
		IKChain& leftLegChain = m_ikChains[0];
		auto positions = m_animationConfigs[0].getModelSpacePositions(false);
		float legLenght = 0;
		for (int i = leftLegChain.m_joints.size() - 1; 0 < i ; i--) {
			JointIndex currentJ = leftLegChain.m_joints[i];
			JointIndex parentJ = leftLegChain.m_joints[i-1];
			legLenght += glm::distance(positions[currentJ], positions[parentJ]);
		}
		m_rigHeight = legLenght * 2;
	}

	IKChain IKRig::buildIKChain(ChainEnds chainEnds, std::string chainName) {
		MONA_ASSERT(chainEnds.baseJointName.empty() || chainEnds.endEffectorName.empty(), "IKRig: Joint names cannot be empty!");
		MONA_ASSERT(chainEnds.baseJointName != chainEnds.endEffectorName, "IKRig: Base joint and end effector must be different!");
		auto jointNames = getJointNames();
		IKChain ikChain;
		ikChain.m_name = chainName;
		JointIndex eeIndex = funcUtils::findIndex(jointNames, chainEnds.endEffectorName);
		if (eeIndex != -1) {
			int chainBaseIndex = -1;
			IKNode* currentNode = &m_nodes[eeIndex];
			while (currentNode != nullptr) {
				if (currentNode->m_jointName == chainEnds.baseJointName) {
					chainBaseIndex = currentNode->m_jointIndex;
					break;
				}
				// la joint correspondiente a la base de la cadena no se guarda para ser modificada mediante IK
				// , ya que esta es una articulacion que se considera fija
				ikChain.m_joints.insert(ikChain.m_joints.begin(), currentNode->m_jointIndex);
				currentNode = currentNode->m_parent;
			}
			if (chainBaseIndex == -1) {
				MONA_LOG_ERROR("IKRig: base joint and end effector were not on the same chain!");
			}
		}
		else {
			MONA_LOG_ERROR("IKRig: Did not find an end effector named {0}", chainEnds.endEffectorName);
		}
		return ikChain;
	}

	std::vector<std::pair<JointIndex, glm::fquat>> IKRig::calculateRotations(AnimationIndex animIndex) {
		return m_inverseKinematics.solveIKChains(animIndex);
	}

}
