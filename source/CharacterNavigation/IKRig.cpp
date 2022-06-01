#include "IKRig.hpp"
#include "../Core/Log.hpp"
#include "../Utilities/FuncUtils.hpp"


namespace Mona {

	glm::vec3 vec3Lerp(glm::vec3 v1, glm::vec3 v2, float fraction) {
		return v1 + (v2 - v1) * fraction;
	}

	IKRig::IKRig(std::shared_ptr<AnimationClip> baseAnim, RigData rigData, InnerComponentHandle rigidBodyHandle,
		InnerComponentHandle skeletalMeshHandle, AnimationController* animController) : m_animationController(animController),
		m_rigidBodyHandle(rigidBodyHandle), m_skeletalMeshHandle(skeletalMeshHandle)
	{
		if (!baseAnim->m_stableRotations) {
			MONA_LOG_ERROR("IKRig: Animation must have stable rotations (fixed scales and positions per joint).");
			return;
		}
		m_animations.push_back(baseAnim);
		m_animConfigurations.push_back(IKRigConfig(baseAnim, 0));
		m_skeleton = baseAnim->GetSkeleton();

		std::shared_ptr<AnimationClip> staticData = m_animations[0];
		m_currentAnim = 0;
		auto topology = GetTopology();
		auto jointNames = GetJointNames();

		// construimos el ikRig
		m_nodes = std::vector<IKNode>(jointNames.size());
		m_nodes[0] = IKNode(jointNames[0], 0);
		for (int i = 1; i < jointNames.size(); i++) {
			m_nodes[i] = IKNode(jointNames[i], i, &m_nodes[topology[i]]);
		}

		
		std::vector<ChainEnds> dataArr = { rigData.leftLeg, rigData.rightLeg, rigData.leftFoot, rigData.rightFoot };
		std::vector<std::pair<int, int>*> nodeTargets = { &m_leftLeg, &m_rightLeg, &m_leftFoot, &m_rightFoot };
		for (int i = 0; i < dataArr.size(); i++) { // construccion de las cadenas principales
			int eeIndex = funcUtils::findIndex(jointNames, dataArr[i].endEffectorName);
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
					MONA_LOG_ERROR("IKRig: Starting joint and end effector were not on the same chain!"); 
				}
			}
		}

		// setear constraints y pesos
		for (int i = 0; i < m_nodes.size(); i++) {
			JointData currData = rigData.jointData[m_nodes[i].m_jointName];
			if (currData.enableIKRotation) {
				m_nodes[i].m_minAngle = currData.minAngle;
				m_nodes[i].m_maxAngle = currData.maxAngle;
				m_nodes[i].m_weight = currData.weight;
			}
		}

		// crear validador
		m_configValidator = IKRigConfigValidator(&m_nodes, &topology);
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
		for (int i = 0; i < m_animations.size(); i++) {
			if (m_animations[i]->GetAnimationName() == animationClip->GetAnimationName()) {
				MONA_LOG_WARNING("IKRig: Animation {0} for model {1} had already been added", 
					animationClip->GetAnimationName(), m_skeleton->GetModelName());
				return;
			}
		}
		m_animations.push_back(animationClip);
	}
	int IKRig::removeAnimation(std::shared_ptr<AnimationClip> animationClip) {
		for (int i = 0; i < m_animations.size(); i++) {
			if (m_animations[i] == animationClip) {
				m_animations.erase(m_animations.begin() + i);
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

	IKRigConfig* IKRig::getAnimConfig(float time, AnimIndex animIndex) {
		auto anim = m_animations[animIndex];
		IKRigConfig* configPtr = &m_animConfigurations[animIndex];
		glm::fquat rot;
		for (int i = 0; i < m_nodes.size(); i++) {
			rot = anim->GetRotation(time, i, m_animationController->GetIsLooping());
			configPtr->baseJointRotations[i].setRotation(rot);
			configPtr->dynamicJointRotations[i].setRotation(rot);
		}
		return configPtr;
	}

	std::vector<glm::vec3> IKRig::modelSpacePositions(const IKRigConfig& rigConfig) {
		std::vector<glm::vec3> modelSpacePos(m_nodes.size());
		auto anim = m_animations[rigConfig.animIndex];
		modelSpacePos[0] = { 0,0,0 };
		// root
		modelSpacePos[0] = modelSpacePos[0] * glm::toMat3(rigConfig.baseJointRotations[0].getQuatRotation()) + rigConfig.jointPositions[0];
		for (int i = 1; i < m_nodes.size(); i++) {
			modelSpacePos[i] = modelSpacePos[GetTopology()[i]] * glm::toMat3(rigConfig.baseJointRotations[i].getQuatRotation()) + rigConfig.jointPositions[i];
		}
		return modelSpacePos;
	}
	glm::vec3 IKRig::getCenterOfMass(const IKRigConfig& rigConfig) {
		std::vector<glm::vec3> modelSpacePos = modelSpacePositions(rigConfig);
		std::vector<glm::vec3> segmentCenters(m_nodes.size());
		int segNum = 0;
		float totalSegLength = 0;
		for (int i = 1; i < m_nodes.size(); i++) {
			glm::vec3 v1 = modelSpacePos[i];
			glm::vec3 v2 = modelSpacePos[GetTopology()[i]];
			float frac = m_nodes[GetTopology()[i]].m_weight / (m_nodes[i].m_weight + m_nodes[GetTopology()[i]].m_weight);
			float segLength = glm::distance(v1, v2);
			segmentCenters[i] = vec3Lerp(v1, v2, frac) * segLength;
			segNum += 1;
			totalSegLength += segLength;
		}
		glm::vec3 centerOfMass = { 0,0,0 };
		for (int i = 1; i < segmentCenters.size(); i++) {
			centerOfMass += segmentCenters[i];
		}
		return centerOfMass / (segNum * totalSegLength);
	}

}
