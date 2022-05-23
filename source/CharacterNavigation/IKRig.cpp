#include "IKRig.hpp"
#include "../Core/Log.hpp"
#include "../Utilities/FuncUtils.hpp"


namespace Mona {

	Vector3f vec3Lerp(Vector3f v1, Vector3f v2, float fraction) {
		return v1 + (v2 - v1) * fraction;
	}

	IKRig::IKRig(std::shared_ptr<BVHData> baseAnim, RigData rigData, InnerComponentHandle rigidBodyHandle,
		InnerComponentHandle skeletalMeshHandle) {
		m_bvhAnims.push_back(baseAnim);
		m_rigidBodyHandle = rigidBodyHandle;
		m_skeletalMeshHandle = skeletalMeshHandle;

		std::shared_ptr<BVHData> staticData = m_bvhAnims[0];
		m_currentAnim = 0;
		m_topology = staticData->getTopology();
		m_jointNames = staticData->getJointNames();
		m_offsets = staticData->getOffsets();
 
		MONA_ASSERT(m_topology.size() == m_jointNames.size(), "IKRig: Topology and jointNames arrays should have the same size");

		// construimos el ikRig
		m_nodes = std::vector<IKNode>(m_jointNames.size());
		m_nodes[0] = IKNode(m_jointNames[0], 0);
		for (int i = 1; i < m_jointNames.size(); i++) {
			m_nodes[i] = IKNode(m_jointNames[i], i, &m_nodes[m_topology[i]]);
		}

		
		std::vector<ChainEnds> dataArr = { rigData.leftLeg, rigData.rightLeg, rigData.leftFoot, rigData.rightFoot };
		std::vector<std::pair<int, int>*> nodeTargets = { &m_leftLeg, &m_rightLeg, &m_leftFoot, &m_rightFoot };
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
		m_configValidator = IKRigConfigValidator(&m_nodes, &m_topology);
	}

	void IKRig::addAnimation(std::shared_ptr<AnimationClip> animationClip, ComponentManager<SkeletalMeshComponent>* skeletalMeshManagerPtr) {
		std::shared_ptr<Skeleton> skeletonPtr = skeletalMeshManagerPtr->GetComponentPointer(m_skeletalMeshHandle)->GetSkeleton();
		if (animationClip->GetSkeleton() != skeletonPtr) {
			MONA_LOG_ERROR("IKRig: Input animation does not correspond to base skeleton.");
			return;
		}
		std::shared_ptr<BVHData> bvhPtr = BVHManager::GetInstance().readBVH(animationClip);
		for (int i = 0; i < m_bvhAnims.size(); i++) {
			if (m_bvhAnims[i]->getModelName() == bvhPtr->getModelName() && m_bvhAnims[i]->getAnimName() == bvhPtr->getAnimName()) {
				MONA_LOG_WARNING("IKRig: Animation {0} for model {1} had already been added", bvhPtr->getAnimName(), bvhPtr->getModelName());
				return;
			}
		}
		if (!(bvhPtr->getJointNames() == m_jointNames)) {
			MONA_LOG_ERROR("IKRig: jointNames of new animation must fit base animation");
			return;
		}
		if (!(bvhPtr->getTopology() == m_topology)) {
			MONA_LOG_ERROR("IKRig: topology of new animation must fit base animation");
			return;
		}
		
		for (int i = 0; i < m_offsets.size(); i++) {
			if (!m_offsets[i].isApprox(bvhPtr->getOffsets()[i])) {
				MONA_LOG_ERROR("IKRig: offsets of new animation must fit base animation");
				return;
			}
		}
		m_bvhAnims.push_back(bvhPtr);
	}
	int IKRig::removeAnimation(std::shared_ptr<AnimationClip> animationClip) {
		std::shared_ptr<BVHData> bvhPtr = BVHManager::GetInstance().getBVHData(animationClip);
		if (bvhPtr != nullptr) {
			for (int i = 0; i < m_bvhAnims.size(); i++) {
				if (m_bvhAnims[i]->getModelName() == bvhPtr->getModelName() &&
					m_bvhAnims[i]->getAnimName() == bvhPtr->getAnimName()) {
					m_bvhAnims.erase(m_bvhAnims.begin() + i);
					return i;
				}
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

	IKRigConfig IKRig::getBVHConfig(int frame, BVHIndex animIndex) {
		auto anim = m_bvhAnims[animIndex];
		IKRigConfig rigConfig;
		rigConfig = std::vector<JointRotation>(m_nodes.size());
		for (int i = 0; i < m_nodes.size(); i++) {
			rigConfig[i].setRotation(anim->getDynamicRotations()[frame][i]);
		}
		return rigConfig;
	}

	IKRigConfig IKRig::createDynamicConfig() {
		IKRigConfig rigConfig;
		rigConfig = std::vector<JointRotation>(m_nodes.size());
		return rigConfig;
	}

	std::vector<Vector3f> IKRig::modelSpacePositions(IKRigConfig rigConfig) {
		std::vector<Vector3f> modelSpacePos(m_nodes.size());
		modelSpacePos[0] = { 0,0,0 };
		// root
		modelSpacePos[0] = modelSpacePos[0] * rigConfig[0].getQuatRotation().toRotationMatrix() + m_offsets[0];
		for (int i = 1; i < m_nodes.size(); i++) {
			modelSpacePos[i] = modelSpacePos[m_topology[i]] * rigConfig[i].getQuatRotation().toRotationMatrix() + m_offsets[i];
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
			Vector3f v2 = modelSpacePos[m_topology[i]];
			float frac = m_nodes[m_topology[i]].m_weight / (m_nodes[i].m_weight + m_nodes[m_topology[i]].m_weight);
			float segLength = (v2 - v1).norm();
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
