#include "IKRigBase.hpp"
#include "Kinematics.hpp"
#include "../Animation/AnimationClip.hpp"
#include "../Animation/Skeleton.hpp"

namespace Mona{

	IKAnimation::IKAnimation(std::shared_ptr<AnimationClip> animationClip, AnimationType animationType, 
		AnimationIndex animationIndex, ForwardKinematics* forwardKinematics) {
		m_animationClip = animationClip;
		m_animationType = animationType;
		std::vector<JointIndex>const& topology = m_animationClip->GetSkeleton()->m_parentIndices;
		std::vector<JointIndex> trackJointIndices = m_animationClip->m_trackJointIndices;
		std::sort(trackJointIndices.begin(), trackJointIndices.end());
		m_jointIndices = trackJointIndices;
		int frameNum = animationClip->m_animationTracks[0].rotationTimeStamps.size();
		int jointNum = animationClip->m_animationTracks.size();
		int totalJointNum = topology.size();
		m_baseJointRotations.resize(frameNum);
		for (FrameIndex i = 0; i < frameNum; i++) {
			m_baseJointRotations[i] = std::vector<JointRotation>(totalJointNum, JointRotation());
			for (int j = 0; j < jointNum; j++) {
				JointIndex jIndex = getJointIndices()[j];
				int trackIndex = animationClip->GetTrackIndex(jIndex);
				m_baseJointRotations[i][jIndex] = JointRotation(animationClip->m_animationTracks[trackIndex].rotations[i]);
			}
		}
		m_variableJointRotations = m_baseJointRotations[0];
		m_animIndex = animationIndex;
		m_forwardKinematics = forwardKinematics;
		m_savedAngles = std::vector<LIC<1>>(totalJointNum);
		
	}
	void IKAnimation::setVariableJointRotations(FrameIndex frame) {
		m_variableJointRotations = m_baseJointRotations[frame];
	}

	void IKAnimation::refreshSavedAngles(JointIndex jointIndex) {
		std::vector<glm::vec1> jointAngles;
		float currentFrameRepTime = getReproductionTime(m_currentFrameIndex);
		float repCountOffset_next = m_currentFrameIndex < m_nextFrameIndex ? 0 : 1;
		float nextFrameRepTime = getReproductionTime(m_nextFrameIndex, repCountOffset_next);
		float currFrameBaseAngle = m_baseJointRotations[m_currentFrameIndex][jointIndex].getRotationAngle();
		float nextFrameBaseAngle = m_baseJointRotations[m_nextFrameIndex][jointIndex].getRotationAngle();
		m_savedAngles[jointIndex] = LIC<1>({ glm::vec1(currFrameBaseAngle), glm::vec1(nextFrameBaseAngle) }, { currentFrameRepTime, nextFrameRepTime });
	}

	std::vector<glm::mat4> IKAnimation::getEEListModelSpaceVariableTransforms(std::vector<JointIndex> eeList, std::vector<glm::mat4>* outJointSpaceTransforms) {
		return m_forwardKinematics->EEListCustomSpaceVariableTransforms(eeList, glm::identity<glm::mat4>(), m_animIndex, outJointSpaceTransforms);
	}
	std::vector<glm::mat4> IKAnimation::getEEListCustomSpaceTransforms(std::vector<JointIndex> eeList, glm::mat4 baseTransform, float reproductionTime, std::vector<glm::mat4>* outJointSpaceTransforms) {
		return m_forwardKinematics->EEListCustomSpaceTransforms(eeList, baseTransform, m_animIndex, reproductionTime, outJointSpaceTransforms);
	}

	const std::vector<float>& IKAnimation::getTimeStamps() {
		return m_animationClip->m_animationTracks[0].rotationTimeStamps;
	}

	const glm::vec3& IKAnimation::getJointScale(JointIndex joint) const{
		return m_animationClip->m_animationTracks[m_animationClip->GetTrackIndex(joint)].scales[0];
	}
	const glm::vec3& IKAnimation::getJointPosition(JointIndex joint) const{
		return m_animationClip->m_animationTracks[m_animationClip->GetTrackIndex(joint)].positions[0];
	}

	float IKAnimation::getReproductionTime(FrameIndex frame, int repCountOffset) {
		MONA_ASSERT(0 <= frame && frame < getFrameNum(), "IKAnimation: FrameIndex outside of range.");
		return (m_reproductionCount + repCountOffset) * m_animationClip->GetDuration() + getTimeStamps()[frame];
	}
	float IKAnimation::getAnimationDuration() {
		return m_animationClip->GetDuration(); 
	}

	float IKAnimation::adjustAnimationTime(float extendedAnimationTime) {
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

	float IKAnimation::getReproductionTime(float extendedAnimationTime, int repCountOffset) {
		// llevar el tiempo al rango correcto
		extendedAnimationTime = adjustAnimationTime(extendedAnimationTime);
		return (m_reproductionCount + repCountOffset) * m_animationClip->GetDuration() + extendedAnimationTime;
	}

	float IKAnimation::getAnimationTime(FrameIndex frame) { 
		MONA_ASSERT(0 <= frame && frame < getFrameNum(), "IKAnimation: FrameIndex outside of range.");
		return getTimeStamps()[frame];
	}

	float IKAnimation::getAnimationTime(float reproductionTime) {
		while (reproductionTime < 0) {
			reproductionTime += m_animationClip->GetDuration();
		}
		return fmod(reproductionTime, m_animationClip->GetDuration());
	}

	FrameIndex IKAnimation::getFrame(float extendedAnimationTime) {
		// llevar el tiempo al rango correcto
		extendedAnimationTime = adjustAnimationTime(extendedAnimationTime);
		// si esta en torno a un frame
		float epsilon = (getAnimationDuration()/getFrameNum()) / 1000;
		for (FrameIndex i = 0; i < getTimeStamps().size(); i++) {
			if (abs(extendedAnimationTime - getTimeStamps()[i]) < epsilon) {
				return i;
			}
		}
		for (FrameIndex i = 0; i < getTimeStamps().size()-1; i++) {
			if (getTimeStamps()[i] <= extendedAnimationTime && extendedAnimationTime < getTimeStamps()[i + 1]) {
				return i;
			}
		}
		return -1;
	}

	bool IKAnimation::hasJoint(JointIndex joint) {
		for (int i = 0; i < getJointIndices().size(); i++) {
			if (getJointIndices()[i] == joint) {
				return true;
			}
		}
		return false;
	}

	bool IKAnimation::isMovementFixed() {
		for (int i = 0; i < m_eeTrajectoryData.size(); i++) {
			if (m_eeTrajectoryData[i].isTargetFixed()) {
				return true;
			}
		}
		return false;
	}

	void IKAnimation::refresh() {
		for (int i = 0; i < m_eeTrajectoryData.size(); i++) {
			m_eeTrajectoryData[i].refresh();
		}
		m_hipTrajectoryData.refresh();
		m_fixedMovementFrame = -1;
		for (int i = 0; i < m_savedAngles.size(); ++i) {
			refreshSavedAngles(i);
		}
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