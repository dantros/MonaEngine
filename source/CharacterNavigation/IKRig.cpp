#include "IKRig.hpp"
#include "../Core/Log.hpp"
#include "../Core/FuncUtils.hpp"
#include "../Animation/Skeleton.hpp"
#include "../Animation/AnimationClip.hpp"


namespace Mona {

	IKRig::IKRig(std::shared_ptr<Skeleton> skeleton,
		RigData rigData, InnerComponentHandle transformHandle)
	{
		m_skeleton = skeleton;
		m_transformHandle = transformHandle;

		std::vector<int> const& topology = getTopology();
		std::vector<std::string>const& jointNames = getJointNames();
		int jointNum = topology.size();

		// angulo de rotacion inicial
		m_rotationAngle = glm::radians(rigData.initialRotationAngle);
		// seteo de la cadera
		JointIndex hipInd = funcUtils::findIndex(jointNames, rigData.hipJointName);
		MONA_ASSERT(hipInd != -1, "IKRig: Input hip joint name was not a correct joint name.");
		m_hipJoint = hipInd;	
		
		// construccion de las cadenas principales
		m_ikChains = { buildIKChain(rigData.leftLeg, "leftLeg"), buildIKChain(rigData.rightLeg, "rightLeg") };
		// seteo de cadenas opuestas
		m_ikChains[0].m_opposite = 1;
		m_ikChains[1].m_opposite = 0;

		for (int i = 0; i < m_ikChains.size(); i++) {
			IKChain chain = m_ikChains[i];
			for (int j = 0; j < chain.m_joints.size(); j++) {
				MONA_ASSERT(funcUtils::findIndex(m_ikChains[chain.m_opposite].m_joints, chain.m_joints[j]) == -1,
					"IKRig: Opposite chain joints cannot overlap.");
				MONA_ASSERT(chain.m_joints[j] != m_hipJoint, "IKRig: Leg chains cannot contain the hip joint.");
			}
		}

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
		m_rigScale = rigData.scale;
		m_initialPosition = rigData.initialPosition;
	}

	void IKRig::init() {
		m_inverseKinematics = InverseKinematics(this);
		m_inverseKinematics.init();
		m_forwardKinematics = ForwardKinematics(this);
		m_trajectoryGenerator = TrajectoryGenerator(this);
		m_trajectoryGenerator.init();
		m_ikAnimations.reserve(MAX_EXPECTED_NUMBER_OF_ANIMATIONS_PER_IKRIG);
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

	std::vector<std::pair<JointIndex, float>> IKRig::calculateRotationAngles(AnimationIndex animIndex) {
		return m_inverseKinematics.solveIKChains(animIndex);
	}


	void IKRig::calculateTrajectories(AnimationIndex animIndex, ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		m_trajectoryGenerator.generateNewTrajectories(animIndex, transformManager, staticMeshManager);
	}

	void IKRig::fixAnimation(AnimationIndex animIndex, FrameIndex fixedFrame) {
		IKAnimation& ikAnim = m_ikAnimations[animIndex];
		std::shared_ptr<AnimationClip> animClip = ikAnim.m_animationClip;
		for (ChainIndex i = 0; i < m_ikChains.size(); i++) {
			IKChain& ikChain = m_ikChains[i];
			for (int j = 0; j < ikChain.m_joints.size(); j++) {
				JointIndex jIndex = ikChain.m_joints[j];
				auto& track = animClip->m_animationTracks[animClip->GetTrackIndex(jIndex)];
				for (FrameIndex k = 0; k < ikAnim.getFrameNum(); k++) {
					track.rotations[k] = track.rotations[fixedFrame];
				}
			}
		}
		auto& hipTrack = animClip->m_animationTracks[animClip->GetTrackIndex(m_hipJoint)];
		for (FrameIndex i = 0; i < ikAnim.getFrameNum(); i++) {
			hipTrack.rotations[i] = hipTrack.rotations[fixedFrame];
		}
	}
	void IKRig::resetAnimation(AnimationIndex animIndex) {
		IKAnimation& ikAnim = m_ikAnimations[animIndex];
		std::shared_ptr<AnimationClip> animClip = ikAnim.m_animationClip;
		for (ChainIndex i = 0; i < m_ikChains.size(); i++) {
			IKChain& ikChain = m_ikChains[i];
			for (int j = 0; j < ikChain.m_joints.size(); j++) {
				JointIndex jIndex = ikChain.m_joints[j];
				auto& track = animClip->m_animationTracks[animClip->GetTrackIndex(jIndex)];
				for (FrameIndex k = 0; k < ikAnim.getFrameNum(); k++) {
					std::vector<JointRotation>const& baseRotations = ikAnim.getOriginalJointRotations(k);
					track.rotations[k] = baseRotations[jIndex].getQuatRotation();
				}
			}
		}
		auto& hipTrack = animClip->m_animationTracks[animClip->GetTrackIndex(m_hipJoint)];
		for (FrameIndex i = 0; i < ikAnim.getFrameNum(); i++) {
			std::vector<JointRotation>const& baseRotations = ikAnim.getOriginalJointRotations(i);
			hipTrack.rotations[i] = baseRotations[m_hipJoint].getQuatRotation();
		}
	}


	std::vector<JointIndex> IKRig::getJointChildren(JointIndex jointIndex) {
		std::vector<JointIndex> children = {};
		for (int i = 0; i < getTopology().size(); i++) {
			if (getTopology()[i] == jointIndex) {
				children.push_back(i);
			}
		}
		return children;
	}


}
