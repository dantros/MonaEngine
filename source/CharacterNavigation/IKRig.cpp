#include "IKRig.hpp"
#include "../Core/Log.hpp"
#include "../Core/FuncUtils.hpp"
#include "../Animation/Skeleton.hpp"


namespace Mona {

	IKRig::IKRig(std::shared_ptr<Skeleton> skeleton,
		RigData rigData, InnerComponentHandle transformHandle)
	{
		m_skeleton = skeleton;
		m_transformHandle = transformHandle;

		std::vector<int> const& topology = getTopology();
		std::vector<std::string>const& jointNames = getJointNames();
		int jointNum = topology.size();

		// seteo de la cadera
		JointIndex hipInd = funcUtils::findIndex(jointNames, rigData.hipJointName);
		MONA_ASSERT(hipInd != -1, "IKRig: Input hip joint name was not a correct joint name.");
		m_hipJoint = hipInd;	
		
		// construccion de las cadenas principales
		m_ikChains = { buildIKChain(rigData.leftLeg, "leftLeg"), buildIKChain(rigData.rightLeg, "rightLeg") };
		// seteo de cadenas opuestas
		m_ikChains[0].m_opposite = 1;
		m_ikChains[1].m_opposite = 0;

		// setear el la altura del rig
		IKChain& leftLegChain = m_ikChains[0];
		std::vector<glm::vec3> positions(jointNum);
		for (JointIndex i = 0; i < jointNum; i++) {
			positions[i] = glm::inverse(skeleton->GetInverseBindPoseMatrix(i)) * glm::vec4(0, 0, 0, 1);
		}
		float legLenght = 0;
		for (int i = leftLegChain.m_joints.size() - 1; 0 < i ; i--) {
			JointIndex currentJ = leftLegChain.m_joints[i];
			JointIndex parentJ = leftLegChain.m_joints[i-1];
			legLenght += glm::distance(positions[currentJ], positions[parentJ]);
		}
		m_rigHeight = legLenght * 2;
	}

	void IKRig::init() {
		std::vector<ChainIndex> ikChains;
		for (ChainIndex i = 0; i < m_ikChains.size(); i++) {
			ikChains.push_back(i);
		}
		m_inverseKinematics = InverseKinematics(this, ikChains);
		m_inverseKinematics.init();
		m_forwardKinematics = ForwardKinematics(this);
		m_trajectoryGenerator = TrajectoryGenerator(this);
		m_trajectoryGenerator.init();
		m_animationConfigs.reserve(MAX_EXPECTED_NUMBER_OF_ANIMATIONS_PER_IKRIG);
	}

	const std::vector<int>& IKRig::getTopology() const { 
		return m_skeleton->m_parentIndices; 
	};
	const std::vector<std::string>& IKRig::getJointNames() const { 
		return m_skeleton->m_jointNames; 
	};

	IKChain IKRig::buildIKChain(ChainEnds chainEnds, std::string chainName) {
		MONA_ASSERT(!(chainEnds.baseJointName.empty() || chainEnds.endEffectorName.empty()), "IKRig: Joint names cannot be empty!");
		MONA_ASSERT(chainEnds.baseJointName != chainEnds.endEffectorName, "IKRig: Base joint and end effector must be different!");
		auto jointNames = getJointNames();
		auto topology = getTopology();
		IKChain ikChain;
		ikChain.m_name = chainName;
		JointIndex eeIndex = funcUtils::findIndex(jointNames, chainEnds.endEffectorName);
		MONA_ASSERT(eeIndex != -1, "IKRig: Did not find an end effector named {0}", chainEnds.endEffectorName);
		int chainBaseIndex = -1;
		JointIndex jIndex = eeIndex;
		while (jIndex != -1) {
			ikChain.m_joints.insert(ikChain.m_joints.begin(), jIndex);
			if (jointNames[jIndex] == chainEnds.baseJointName) {
				chainBaseIndex = jIndex;
				break;
			}
			jIndex = topology[jIndex];
		}
		MONA_ASSERT(chainBaseIndex != -1, "IKRig: base joint and end effector were not on the same chain!");
		ikChain.m_parentJoint = topology[chainBaseIndex];
		return ikChain;
	}

	std::vector<std::pair<JointIndex, glm::fquat>> IKRig::calculateRotations(AnimationIndex animIndex, FrameIndex targetFrame) {
		return m_inverseKinematics.solveIKChains(animIndex, targetFrame);
	}


	void IKRig::calculateTrajectories(AnimationIndex animIndex, ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		m_trajectoryGenerator.generateNewTrajectories(animIndex, transformManager, staticMeshManager);
	}

}
