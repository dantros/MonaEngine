#include "IKRig.hpp"
#include "../Core/Log.hpp"
#include "../Utilities/FuncUtils.hpp"


namespace Mona {

	Vector3f vec3Lerp(Vector3f v1, Vector3f v2, float fraction) {
		return v1 + (v2 - v1) * fraction;
	}

	IKRig::IKRig(std::shared_ptr<AnimationClip> baseAnim, RigData rigData, InnerComponentHandle rigidBodyHandle,
		InnerComponentHandle skeletalMeshHandle) {
		m_animations.push_back(baseAnim);
		m_rigidBodyHandle = rigidBodyHandle;
		m_skeletalMeshHandle = skeletalMeshHandle;
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

	Vector3f IKRig::getLinearVelocity(ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr) {
		auto bodyVel = rigidBodyManagerPtr->GetComponentPointer(m_rigidBodyHandle)->GetLinearVelocity();
		return { bodyVel[0], bodyVel[1], bodyVel[2] };
	}

	void IKRig::setLinearVelocity(Vector3f velocity, ComponentManager<RigidBodyComponent>* rigidBodyManagerPtr) {
		rigidBodyManagerPtr->GetComponentPointer(m_rigidBodyHandle)->SetLinearVelocity({velocity[0], velocity[1], velocity[2]});
	}

	IKRigConfig IKRig::getAnimConfig(int frame, AnimIndex animIndex) {
		auto anim = m_animations[animIndex];
		IKRigConfig rigConfig;
		rigConfig.jointRotations = std::vector<JointRotation>(m_nodes.size());
		rigConfig.animIndex = animIndex;
		for (int i = 0; i < m_nodes.size(); i++) {
			rigConfig.jointRotations[i].setRotation(anim->GetRotation(frame, i));
		}
		return rigConfig;
	}

	IKRigConfig IKRig::createDynamicConfig(int animIndex) {
		IKRigConfig rigConfig;
		rigConfig.animIndex = animIndex;
		rigConfig.jointRotations = std::vector<JointRotation>(m_nodes.size());
		return rigConfig;
	}

	std::vector<Vector3f> IKRig::modelSpacePositions(IKRigConfig rigConfig) {
		std::vector<Vector3f> modelSpacePos(m_nodes.size());
		auto anim = m_animations[rigConfig.animIndex];
		modelSpacePos[0] = { 0,0,0 };
		// root
		modelSpacePos[0] = modelSpacePos[0] * glm::toMat3(rigConfig.jointRotations[0].getQuatRotation()) + anim->GetPosition(0, 0);
		for (int i = 1; i < m_nodes.size(); i++) {
			modelSpacePos[i] = modelSpacePos[GetTopology()[i]] * glm::toMat3(rigConfig.jointRotations[i].getQuatRotation()) + anim->GetPosition(0, i);
		}
		return modelSpacePos;
	}
	Vector3f IKRig::getCenterOfMass(IKRigConfig rigConfig) {
		std::vector<Vector3f> modelSpacePos = modelSpacePositions(rigConfig);
		std::vector<Vector3f> segmentCenters(m_nodes.size());
		int segNum = 0;
		float totalSegLength = 0;
		for (int i = 1; i < m_nodes.size(); i++) {
			Vector3f v1 = modelSpacePos[i];
			Vector3f v2 = modelSpacePos[GetTopology()[i]];
			float frac = m_nodes[GetTopology()[i]].m_weight / (m_nodes[i].m_weight + m_nodes[GetTopology()[i]].m_weight);
			float segLength = glm::distance(v1, v2);
			segmentCenters[i] = vec3Lerp(v1, v2, frac) * segLength;
			segNum += 1;
			totalSegLength += segLength;
		}
		Vector3f centerOfMass = { 0,0,0 };
		for (int i = 1; i < segmentCenters.size(); i++) {
			centerOfMass += segmentCenters[i];
		}
		return centerOfMass / (segNum * totalSegLength);
	}

}
