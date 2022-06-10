#include "IKRig.hpp"
#include "../Core/Log.hpp"
#include "../Core/FuncUtils.hpp"


namespace Mona {

	glm::vec3 vec3Lerp(glm::vec3 v1, glm::vec3 v2, float fraction) {
		return v1 + (v2 - v1) * fraction;
	}

	IKRig::IKRig(std::shared_ptr<AnimationClip> baseAnim, RigData rigData, InnerComponentHandle rigidBodyHandle,
		InnerComponentHandle skeletalMeshHandle, AnimationController* animController, ComponentManager<SkeletalMeshComponent>* skeletalMeshManagerPtr) : 
		m_animationController(animController), m_rigidBodyHandle(rigidBodyHandle), m_skeletalMeshHandle(skeletalMeshHandle)
	{
		m_skeleton = baseAnim->GetSkeleton();
		m_forwardKinematics = ForwardKinematics(this);
		addAnimation(baseAnim, skeletalMeshManagerPtr);
		m_animations.push_back(baseAnim);
		m_animationConfigs.push_back(IKRigConfig(baseAnim, 0, &m_forwardKinematics));

		std::shared_ptr<AnimationClip> staticData = m_animations[0];
		m_currentAnim = 0;
		auto topology = getTopology();
		auto jointNames = getJointNames();

		// construimos el ikRig
		m_nodes = std::vector<IKNode>(jointNames.size());
		m_nodes[0] = IKNode(jointNames[0], 0);
		for (int i = 1; i < jointNames.size(); i++) {
			m_nodes[i] = IKNode(jointNames[i], i, &m_nodes[topology[i]]);
		}

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
		m_inverseKinematics = InverseKinematics(this, m_ikChains, 0);
	}

	void IKRig::addAnimation(std::shared_ptr<AnimationClip> animationClip, ComponentManager<SkeletalMeshComponent>* skeletalMeshManagerPtr) {
		std::shared_ptr<Skeleton> skeletonPtr = skeletalMeshManagerPtr->GetComponentPointer(m_skeletalMeshHandle)->GetSkeleton();
		if (animationClip->GetSkeleton() != skeletonPtr) {
			MONA_LOG_ERROR("IKRig: Input animation does not correspond to base skeleton.");
			return;
		}
		if(!animationClip->m_stableRotations) {
			MONA_LOG_ERROR("IKRig: Animation must have stable rotations (fixed scales and positions per joint).");
			return;
		}
		if (!animationClip->m_noRootMotion) {
			MONA_LOG_ERROR("IKRig: Animation must have no root motion.");
			return;
		}
		for (int i = 0; i < m_animations.size(); i++) {
			if (m_animations[i]->GetAnimationName() == animationClip->GetAnimationName()) {
				MONA_LOG_WARNING("IKRig: Animation {0} for model {1} had already been added", 
					animationClip->GetAnimationName(), m_skeleton->GetModelName());
				return;
			}
		}
		// descomprimimos las rotaciones de la animacion, repitiendo valores para que todas las articulaciones 
		// tengan el mismo numero de rotaciones
		std::vector<AnimationClip::AnimationTrack>& tracks = animationClip->m_animationTracks;
		int nTracks = tracks.size();
		std::vector<bool> conditions(nTracks);
		std::vector<int> currentTimeIndexes(nTracks);
		std::vector<float> currentTimes(nTracks);
		for (int i = 0; i < nTracks; i++) {
			currentTimeIndexes[i] = 0;
			conditions[i] = currentTimeIndexes[i] < tracks[i].rotationTimeStamps.size();
		}
		while (funcUtils::conditionArray_OR(conditions)) {
			// seteamos el valor del timestamp que le corresponde a cada track
			for (int i = 0; i < nTracks; i++) {
				currentTimes[i] = conditions[i] ? tracks[i].rotationTimeStamps[currentTimeIndexes[i]] : std::numeric_limits<float>::max();
			}
			// encontramos los indices de las tracks que tienen el minimo timestamp actual
			std::vector<int> minTimeIndexes = funcUtils::minValueIndex_multiple<float>(currentTimes); // ordenados ascendentemente
			float currentMinTime = currentTimes[minTimeIndexes[0]];

			int minTimeIndexesIndex = 0;
			for (int i = 0; i < nTracks; i++) {
				if (minTimeIndexesIndex < minTimeIndexes.size() && minTimeIndexes[minTimeIndexesIndex] == i) { // track actual tiene un timestamp minimo
					currentTimeIndexes[i] += 1;
					minTimeIndexesIndex += 1;
				}
				else {
					// si el valor a insertar cae antes del primer timestamp, se replica el ultimo valor del arreglo de rotaciones
					// se asume animacion circular
					int insertOffset = currentTimeIndexes[i];
					int valIndex = currentTimeIndexes[i] > 0 ? currentTimeIndexes[i] - 1 : tracks[i].rotationTimeStamps.size() - 1;
					auto rotIt = tracks[i].rotations.begin() + insertOffset;
					auto timeRotIt = tracks[i].rotationTimeStamps.begin() + insertOffset;
					tracks[i].rotations.insert(rotIt, tracks[i].rotations[valIndex]);
					tracks[i].rotationTimeStamps.insert(timeRotIt, currentMinTime);
					currentTimeIndexes[i] += 1;
				}
			}

			// actualizamos las condiciones
			for (int i = 0; i < nTracks; i++) {
				conditions[i] = currentTimeIndexes[i] < tracks[i].rotationTimeStamps.size();
			}
		}
		m_animations.push_back(animationClip);
		m_animationConfigs.push_back(IKRigConfig(animationClip, m_animations.size() - 1, &m_forwardKinematics));
	}

	int IKRig::removeAnimation(std::shared_ptr<AnimationClip> animationClip) {
		for (int i = 0; i < m_animations.size(); i++) {
			if (m_animations[i] == animationClip) {
				m_animations.erase(m_animations.begin() + i);
				m_animationConfigs.erase(m_animationConfigs.begin() + i);
				return i;
			}
		}
		return -1;
	}

	glm::vec3 IKRig::getLinearVelocity(ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr) {
		auto bodyVel = rigidBodyManagerPtr->GetComponentPointer(m_rigidBodyHandle)->GetLinearVelocity();
		return { bodyVel[0], bodyVel[1], bodyVel[2] };
	}

	void IKRig::setLinearVelocity(glm::vec3 velocity, ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr) {
		rigidBodyManagerPtr->GetComponentPointer(m_rigidBodyHandle)->SetLinearVelocity({velocity[0], velocity[1], velocity[2]});
	}

	void IKRig::setIKRigConfigTime(float time, AnimationIndex animIndex) {
		auto anim = m_animations[animIndex];
		auto& config = m_animationConfigs[animIndex];
		float samplingTime = anim->GetSamplingTime(time, m_animationController->GetIsLooping());
		config.m_currentTime = samplingTime;
		for (int i = 0; i < config.m_timeStamps.size(); i++) {
			if (config.m_timeStamps[i] <= samplingTime) {
				config.m_nextFrameIndex = (config.m_timeStamps.size()) % (i + 1);
				break;
			}
		}
		for (int i = 0; i < config.m_dynamicJointRotations.size(); i++) {
			config.m_dynamicJointRotations[i] = JointRotation(anim->m_animationTracks[i].rotations[config.m_nextFrameIndex]);
		}
	}

	IKChain IKRig::buildIKChain(ChainEnds chainEnds, std::string chainName) {
		MONA_ASSERT(chainEnds.baseJointName != chainEnds.endEffectorName, "Base joint and end effector must be different!");
		auto jointNames = getJointNames();
		IKChain ikChain;
		ikChain.name = chainName;
		int eeIndex = funcUtils::findIndex(jointNames, chainEnds.endEffectorName);
		if (eeIndex != -1) {
			int chainStartIndex = -1;
			IKNode* currentNode = &m_nodes[eeIndex];
			while (currentNode != nullptr) {
				ikChain.joints.insert(ikChain.joints.begin(), currentNode->m_jointIndex);
				if (currentNode->m_jointName == chainEnds.baseJointName) {
					chainStartIndex = currentNode->m_jointIndex;
					break;
				}
				currentNode = currentNode->m_parent;
			}
			if (chainStartIndex == -1) {
				MONA_LOG_ERROR("IKRig: base joint and end effector were not on the same chain!");
			}
		}
		else {
			MONA_LOG_ERROR("IKRig: Did not find an end effector named {0}", chainEnds.endEffectorName);
		}
		return ikChain;
	}

}
