#include "IKRigBase.hpp"
#include "Kinematics.hpp"
#include "../Animation/AnimationClip.hpp"
#include "../Animation/Skeleton.hpp"

namespace Mona{

	IKRigConfig::IKRigConfig(std::shared_ptr<AnimationClip> animation, AnimationType animationType, 
		AnimationIndex animationIndex, ForwardKinematics* forwardKinematics) {
		m_animationClip = animation;
		m_animationType = animationType;
		std::vector<JointIndex>const& topology = m_animationClip->GetSkeleton()->m_parentIndices;
		std::vector<JointIndex> trackJointIndices = m_animationClip->m_trackJointIndices;
		std::sort(trackJointIndices.begin(), trackJointIndices.end());
		m_jointIndices = trackJointIndices;
		int frameNum = animation->m_animationTracks[0].rotationTimeStamps.size();
		int jointNum = animation->m_animationTracks.size();
		int totalJointNum = topology.size();
		m_jointPositions = std::vector<glm::vec3>(totalJointNum, glm::vec3(0));
		m_jointScales = std::vector<glm::vec3>(totalJointNum, glm::vec3(1));
		m_timeStamps = animation->m_animationTracks[0].rotationTimeStamps;
		for (int i = 0; i < jointNum;i++) {
			JointIndex jIndex = getJointIndices()[i];
			int trackIndex = animation->GetTrackIndex(jIndex);
			m_jointPositions[jIndex] = animation->m_animationTracks[trackIndex].positions[0];
			m_jointScales[jIndex] = animation->m_animationTracks[trackIndex].scales[0];
		}
		m_baseJointRotations.resize(frameNum);
		for (FrameIndex i = 0; i < frameNum; i++) {
			m_baseJointRotations[i] = std::vector<JointRotation>(totalJointNum, JointRotation());
			for (int j = 0; j < jointNum; j++) {
				JointIndex jIndex = getJointIndices()[j];
				int trackIndex = animation->GetTrackIndex(jIndex);
				m_baseJointRotations[i][jIndex] = JointRotation(animation->m_animationTracks[trackIndex].rotations[i]);
			}
		}
		m_variableJointRotations = m_baseJointRotations[0];
		m_animIndex = animationIndex;
		m_forwardKinematics = forwardKinematics;
		for (int i = 0; i < totalJointNum; ++i) {
			std::vector<glm::vec1> jointAngles;
			for (FrameIndex j = 0; j < getFrameNum(); j++) {
				jointAngles.push_back(glm::vec1(m_baseJointRotations[j][i].getRotationAngle()));
			}
			m_savedAngles.push_back(LIC<1>(jointAngles, m_timeStamps));
		}
	}
	void IKRigConfig::setVariableJointRotations(FrameIndex frame) {
		m_variableJointRotations = m_baseJointRotations[frame];
	}

	void IKRigConfig::refreshSavedAngles(JointIndex jointIndex) {
		std::vector<glm::vec1> jointAngles;
		float currentRepTime = getCurrentReproductionTime();
		float currentAnimTime = getAnimationTime(currentRepTime);
		for (FrameIndex i = 0; i < getFrameNum(); i++) {
			m_savedAngles[jointIndex].setCurvePoint(i, glm::vec1(m_baseJointRotations[i][jointIndex].getRotationAngle()));
		}
		m_savedAngles[jointIndex].offsetTValues(-m_savedAngles[jointIndex].getTRange()[0]);
		m_savedAngles[jointIndex].offsetTValues(-currentAnimTime + currentRepTime);
	}

	std::vector<glm::mat4> IKRigConfig::getEEListModelSpaceVariableTransforms(std::vector<JointIndex> eeList, std::vector<glm::mat4>* outJointSpaceTransforms) {
		return m_forwardKinematics->EEListCustomSpaceVariableTransforms(eeList, glm::identity<glm::mat4>(), m_animIndex, outJointSpaceTransforms);
	}
	std::vector<glm::mat4> IKRigConfig::getEEListCustomSpaceTransforms(std::vector<JointIndex> eeList, glm::mat4 baseTransform, float reproductionTime, std::vector<glm::mat4>* outJointSpaceTransforms) {
		return m_forwardKinematics->EEListCustomSpaceTransforms(eeList, baseTransform, m_animIndex, reproductionTime, outJointSpaceTransforms);
	}

	float IKRigConfig::getReproductionTime(FrameIndex frame, int repCountOffset) {
		MONA_ASSERT(0 <= frame && frame < m_timeStamps.size(), "IKRigConfig: FrameIndex outside of range.");
		return (m_reproductionCount + repCountOffset) * m_animationClip->GetDuration() + m_timeStamps[frame];
	}
	float IKRigConfig::getAnimationDuration() {
		return m_animationClip->GetDuration(); 
	}

	float IKRigConfig::adjustAnimationTime(float extendedAnimationTime) {
		if (extendedAnimationTime < 0) {
			while (extendedAnimationTime < 0) {
				extendedAnimationTime += getAnimationDuration();
			}
		}
		else if (getAnimationDuration() < extendedAnimationTime) {
			while (getAnimationDuration() < extendedAnimationTime) {
				extendedAnimationTime -= getAnimationDuration();
			}
		}
		return extendedAnimationTime;
	}

	float IKRigConfig::getReproductionTime(float extendedAnimationTime, int repCountOffset) {
		// llevar el tiempo al rango correcto
		extendedAnimationTime = adjustAnimationTime(extendedAnimationTime);
		return (m_reproductionCount + repCountOffset) * m_animationClip->GetDuration() + extendedAnimationTime;
	}

	float IKRigConfig::getAnimationTime(FrameIndex frame) { 
		MONA_ASSERT(0 <= frame && frame < m_timeStamps.size(), "IKRigConfig: FrameIndex outside of range.");
		return m_timeStamps[frame]; 
	}

	float IKRigConfig::getAnimationTime(float reproductionTime) {
		while (reproductionTime < 0) {
			reproductionTime += m_animationClip->GetDuration();
		}
		return fmod(reproductionTime, m_animationClip->GetDuration());
	}

	FrameIndex IKRigConfig::getFrame(float extendedAnimationTime) {
		// llevar el tiempo al rango correcto
		extendedAnimationTime = adjustAnimationTime(extendedAnimationTime);
		// si esta en torno a un frame
		float epsilon = (getAnimationDuration()/getFrameNum()) / 1000;
		for (FrameIndex i = 0; i < m_timeStamps.size(); i++) {
			if (abs(extendedAnimationTime - m_timeStamps[i]) < epsilon) {
				return i;
			}
		}
		for (FrameIndex i = 0; i < m_timeStamps.size()-1; i++) {
			if (m_timeStamps[i] <= extendedAnimationTime && extendedAnimationTime < m_timeStamps[i + 1]) {
				return i;
			}
		}
		return -1;
	}

	bool IKRigConfig::hasJoint(JointIndex joint) {
		for (int i = 0; i < getJointIndices().size(); i++) {
			if (getJointIndices()[i] == joint) {
				return true;
			}
		}
		return false;
	}

	bool IKRigConfig::isMovementFixed() {
		for (int i = 0; i < m_eeTrajectoryData.size(); i++) {
			if (m_eeTrajectoryData[i].isTargetFixed()) {
				return true;
			}
		}
		return false;
	}

	void IKRigConfig::refresh() {
		for (int i = 0; i < m_eeTrajectoryData.size(); i++) {
			m_eeTrajectoryData[i].refresh();
		}
		m_hipTrajectoryData.refresh();
		m_fixedMovementFrame = -1;
		for (int i = 0; i < m_savedAngles.size(); ++i) {
			refreshSavedAngles(i);
		}
	}

	void RigData::setJointMotionRange(std::string jointName, float minAngle, float maxAngle) {
		if (jointName == "") {
			MONA_LOG_ERROR("RigData: jointName cannot be empty string.");
			return;
		}
		if (minAngle <= maxAngle) {
			MONA_LOG_ERROR("RigData: maxAngle must be equal or greater than minAngle.");
			return;
		}
		motionRanges[jointName].minAngle = minAngle;
		motionRanges[jointName].maxAngle = maxAngle;
	}

	MotionRange RigData::getJointMotionRange(std::string jointName) {
		return motionRanges[jointName];
	}


	JointRotation::JointRotation() {
		setRotation(glm::identity<glm::fquat>());
	}
	JointRotation::JointRotation(float rotationAngle, glm::vec3 rotationAxis) {
		setRotation(rotationAngle, rotationAxis);
	}
	JointRotation::JointRotation(glm::fquat quatRotation) {
		setRotation(quatRotation);
	}
	void JointRotation::setRotation(glm::fquat rotation) {
		m_quatRotation = rotation;
		m_rotationAngle = glm::angle(m_quatRotation);
		m_rotationAxis = glm::axis(m_quatRotation);
	}
	void JointRotation::setRotation(float rotationAngle, glm::vec3 rotationAxis) {
		rotationAxis = glm::normalize(rotationAxis);
		m_quatRotation = glm::angleAxis(rotationAngle, rotationAxis);
		m_rotationAngle = rotationAngle;
		m_rotationAxis = rotationAxis;
	}

	void JointRotation::setRotationAngle(float rotationAngle) {
		m_quatRotation = glm::angleAxis(rotationAngle, m_rotationAxis);
		m_rotationAngle = rotationAngle;
	}

	void JointRotation::setRotationAxis(glm::vec3 rotationAxis) {
		rotationAxis = glm::normalize(rotationAxis);
		m_quatRotation = glm::angleAxis(m_rotationAngle, rotationAxis);
		m_rotationAxis = rotationAxis;
	}

}