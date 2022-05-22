#include "IKRig.hpp"
#include "../Core/Log.hpp"
#include "../Utilities/FuncUtils.hpp"


namespace Mona {

	Vector3f vec3Lerp(Vector3f v1, Vector3f v2, float frac) {
		return v1 + (v2 - v1) * frac;
	}

	IKNode::IKNode(std::string jointName, int jointIndex, IKNode* parent, float weight) {
		m_jointName = jointName;
		m_jointIndex = jointIndex;
		m_parent = parent;
		m_weight = weight;
	}

	IKRig::IKRig(std::shared_ptr<BVHData> baseAnim, RigData rigData, InnerComponentHandle rigidBodyHandle,
		InnerComponentHandle skeletalMeshHandle) {
		m_bvhAnims.push_back(baseAnim);
		m_rigidBodyHandle = rigidBodyHandle;
		m_skeletalMeshHandle = skeletalMeshHandle;

		std::shared_ptr<BVHData> staticData = m_bvhAnims[0];
		std::vector<int> topology = staticData->getTopology();
		std::vector<std::string> jointNames = staticData->getJointNames();
		MONA_ASSERT(topology.size() == jointNames.size(), "IKRig: Topology and jointNames arrays should have the same size");
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
					MONA_LOG_ERROR("IKRig: Starting joint and end effector were not on the same chain!"); 
				}
			}
		}

		// setear constraints y pesos
		for (int i = 0; i < m_nodes.size(); i++) {
			JointData currData = rigData.jointData[m_nodes[i].m_jointName];
			if (currData.enableData) {
				m_nodes[i].m_minAngle = currData.minAngle;
				m_nodes[i].m_maxAngle = currData.maxAngle;
				m_nodes[i].m_weight = currData.weight;
			}
		}
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

	std::vector<Vector3f> IKRig::bvhModelSpacePositions(int frame, bool useTargetAnim) {
		std::vector<Vector3f> modelSpacePos(m_nodes.size());
		auto anim = m_currentAnim;
		if (useTargetAnim) {
			if (m_targetAnim == nullptr) {
				MONA_LOG_ERROR("IKRig: target animation was nullptr");
				return modelSpacePos;
			}
			anim = m_targetAnim;
		}
		auto topology = anim->getTopology();
		auto rotations = anim->getDynamicRotations();
		auto offsets = anim->getOffsets();
		modelSpacePos[0] = { 0,0,0 };
		// root
		modelSpacePos[0] = modelSpacePos[0] *rotations[frame][0].toRotationMatrix() + offsets[0];
		for (int i = 1; i < m_nodes.size(); i++) {
			modelSpacePos[i] = modelSpacePos[topology[i]]* rotations[frame][i].toRotationMatrix() + offsets[i];
		}
		return modelSpacePos;
	}
	std::vector<Vector3f> IKRig::dynamicModelSpacePositions(bool useTargetAnim) {
		std::vector<Vector3f> modelSpacePos(m_nodes.size());
		auto anim = m_currentAnim;
		if (useTargetAnim) {
			if (m_targetAnim == nullptr) {
				MONA_LOG_ERROR("IKRig: target animation was nullptr");
				return modelSpacePos;
			}
			anim = m_targetAnim; 
		}
		auto topology = anim->getTopology();
		auto offsets = anim->getOffsets();
		modelSpacePos[0] = { 0,0,0 };
		// root
		modelSpacePos[0] = modelSpacePos[0] * m_nodes[0].m_jointRotation_dmic.getQuatRotation().toRotationMatrix() + offsets[0];
		for (int i = 1; i < m_nodes.size(); i++) {
			modelSpacePos[i] = modelSpacePos[topology[i]] * m_nodes[i].m_jointRotation_dmic.getQuatRotation().toRotationMatrix() + offsets[i];
		}
		return modelSpacePos;
	}
	Vector3f IKRig::_centerOfMass(std::vector<Vector3f> modelSpacePositions) {
		std::vector<Vector3f> modelSpacePos = modelSpacePositions;
		std::vector<Vector3f> segmentCenters(m_nodes.size());
		auto topology = m_currentAnim->getTopology();
		int segNum = 0;
		float totalSegLength = 0;
		for (int i = 1; i < m_nodes.size(); i++) {
			Vector3f v1 = modelSpacePos[i];
			Vector3f v2 = modelSpacePos[topology[i]];
			float frac = m_nodes[topology[i]].m_weight / (m_nodes[i].m_weight + m_nodes[topology[i]].m_weight);
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
	Vector3f IKRig::bvhCenterOfMass(int frame, bool useTargetAnim) {
		std::vector<Vector3f> modelSpacePos = bvhModelSpacePositions(frame, useTargetAnim);
		return _centerOfMass(modelSpacePos);
	}
	Vector3f IKRig::dynamicCenterOfMass(bool useTargetAnim) {
		std::vector<Vector3f> modelSpacePos = dynamicModelSpacePositions(useTargetAnim);
		return _centerOfMass(modelSpacePos);
	}

	void RigData::setJointData(std::string jointName, float minAngle, float maxAngle, float weight, bool enableData) {
		if (jointName == "") {
			MONA_LOG_ERROR("RigData: jointName cannot be empty string.");
			return;
		}
		if (minAngle <= maxAngle) {
			MONA_LOG_ERROR("RigData: maxAngle must be equal or greater than minAngle.");
			return;
		}
		jointData[jointName].minAngle = minAngle;
		jointData[jointName].maxAngle = maxAngle;
		jointData[jointName].weight = weight;
		jointData[jointName].enableData = enableData;
	}

	JointData RigData::getJointData(std::string jointName) {
		return JointData(jointData[jointName]);
	}
	void RigData::enableJointData(std::string jointName, bool enableData) {
		jointData[jointName].enableData = enableData;
	}

	bool RigData::isValid() {
		if (leftLeg.startJointName.empty() || leftLeg.endEffectorName.empty() || rightLeg.startJointName.empty() || rightLeg.endEffectorName.empty()) {
			MONA_LOG_ERROR("RigData: Legs cannot be empty");
			return false;
		}
		if (spine.startJointName.empty() || spine.endEffectorName.empty()) {
			MONA_LOG_ERROR("RigData: Spine cannot be empty");
			return false;
		}
		return true;
	}
	JointRotation::JointRotation() {
		setRotation({ 0,0,0,1 });
	}
	JointRotation::JointRotation(Vector3f rotationAxis, float rotationAngle) {
		setRotation(rotationAxis, rotationAngle);
	}
	JointRotation::JointRotation(Quaternion quatRotation) {
		setRotation(quatRotation);
	}
	void JointRotation::setRotation(Quaternion rotation) {
		m_quatRotation = rotation;
		m_angleAxis = AngleAxis(m_quatRotation);
	}
	void JointRotation::setRotation(Vector3f rotationAxis, float rotationAngle) {
		m_angleAxis = AngleAxis(rotationAngle, rotationAxis);
		m_quatRotation = Quaternion(m_angleAxis);
	}
}
