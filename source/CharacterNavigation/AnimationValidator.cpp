#include "AnimationValidator.hpp"
#include "IKRig.hpp"
#include "../Animation/AnimationClip.hpp"
#include "../Animation/Skeleton.hpp"
#include "../Core/GlmUtils.hpp"



namespace Mona{

    AnimationValidator::AnimationValidator(IKRig* ikRig) {
        m_ikRig = ikRig;
    }

    void AnimationValidator::checkTransforms(std::shared_ptr<AnimationClip> animationClip) {
		// la animacion debe tener globalmente vector front={0,1,0} y up={0,0,1}
		MONA_ASSERT(animationClip->GetSkeleton() == m_ikRig->m_skeleton,
			"AnimationValidator: Input animation does not correspond to base skeleton.");
		for (int i = 0; i < m_ikRig->m_ikAnimations.size(); i++) {
			if (m_ikRig->m_ikAnimations[i].m_animationClip->GetAnimationName() == animationClip->GetAnimationName()) {
				MONA_LOG_WARNING("AnimationValidator: Animation {0} for model {1} had already been added",
					animationClip->GetAnimationName(), m_ikRig->m_skeleton->GetModelName());
				return;
			}
		}
		MONA_ASSERT(animationClip->GetTrackIndex(m_ikRig->m_hipJoint) != -1, "AnimationValidator: Hip joint must be present in input animationClip.");
		for (int i = 0; i < m_ikRig->m_ikChains.size(); i++) {
			JointIndex eeIndex = m_ikRig->m_ikChains[i].getEndEffector();
			MONA_ASSERT(animationClip->GetTrackIndex(eeIndex) != -1,
				"AnimationValidator: Set end effector {0} must be present in input animationClip.",
				m_ikRig->getJointNames()[eeIndex]);
		}

		// Chequar si los escalamientos y traslaciones son constantes por joint.
		// La cadera si puede tener traslaciones variables
		for (int i = 0; i < animationClip->m_animationTracks.size(); i++) {
			auto track = animationClip->m_animationTracks[i];
			JointIndex jIndex = animationClip->m_trackJointIndices[i];
			glm::vec3 basePosition = track.positions[0];
			glm::vec3 baseScale = track.scales[0];
			for (int j = 1; j < track.positions.size(); j++) {
				MONA_ASSERT(glmUtils::areApproxEqual(track.positions[j], basePosition) || jIndex == m_ikRig->m_hipJoint,
					"AnimationValidator: Jointa other than the hip must have fixed translations.");
			}
			for (int j = 1; j < track.scales.size(); j++) {
				MONA_ASSERT(glmUtils::areApproxEqual(track.scales[j], baseScale),
					"AnimationValidator: Animation must have fixed scales per joint.");
			}
			MONA_ASSERT(glmUtils::isApproxUniform(baseScale),
				"AnimationValidator: Animation must have uniform scales.");
		}

		JointIndex parent = m_ikRig->getTopology()[m_ikRig->m_hipJoint];
		while (parent != -1) {
			int trackIndex = animationClip->GetTrackIndex(parent);
			auto track = animationClip->m_animationTracks[trackIndex];
			for (int j = 0; j < track.rotations.size(); j++) {
				MONA_ASSERT(track.rotations[j] == glm::identity<glm::fquat>(),
					"AnimationValidator: Joints above the hip in the hierarchy cannot have rotations.");
			}
			for (int j = 0; j < track.positions.size(); j++) {
				MONA_ASSERT(track.positions[j] == glm::vec3(0),
					"AnimationValidator: Joints above the hip in the hierarchy cannot have translations.");
			}
		}

    }

	void AnimationValidator::checkLegLocalRotationAxes(std::shared_ptr<AnimationClip> animation, IKChain* legChain, 
		glm::vec3 originalUpVector, glm::vec3 originalFrontVector) {
		glm::fquat deltaRotationUp = glmUtils::calcDeltaRotation(originalUpVector, glm::vec3(0, 0, 1), m_ikRig->getRightVector());
		glm::fquat deltaRotationFront = glmUtils::calcDeltaRotation(deltaRotationUp * originalFrontVector, glm::vec3(0, 1, 0), m_ikRig->getUpVector());
		glm::fquat deltaRotation = deltaRotationFront * deltaRotationUp;
		std::vector<JointIndex>const& topology = m_ikRig->getTopology();
		int frameNum = animation->m_animationTracks[0].rotations.size();
		for (int i = 0; i < legChain->getJoints().size() - 1; i++) {
			std::vector<glm::vec3> localRotationAxes;
			JointIndex jIndex = legChain->getJoints()[i];
			for (FrameIndex j = 0; j < frameNum; j++) {
				glm::fquat localRotation = animation->m_animationTracks[animation->GetTrackIndex(jIndex)].rotations[j];
				// aplicamos correccion de orientacion
				glm::vec3 localRotationAxis = deltaRotation * glm::axis(localRotation);
				localRotationAxes.push_back(localRotationAxis);
			}

			// buscamos el eje de rotacion global principal
			glm::vec3 meanRotationAxis(0);
			for (FrameIndex j = 0; j < frameNum; j++) {
				meanRotationAxis += localRotationAxes[j];

			}
			meanRotationAxis /= frameNum;

			if (abs(meanRotationAxis[0]) < 0.7f || 0.3f < abs(meanRotationAxis[1]) || 0.3f < abs(meanRotationAxis[2])) {
				MONA_LOG_WARNING("AnimationValidator:Joint {}'s local rotation axis does not allow for proper IK rotation control.", jIndex);
			}

		}

	}


	void AnimationValidator::correctAnimationOrientation(std::shared_ptr<AnimationClip> animation, 
		glm::vec3 originalUpVector, glm::vec3 originalFrontVector) {
		glm::fquat deltaRotationUp = glmUtils::calcDeltaRotation(originalUpVector, glm::vec3(0, 0, 1), m_ikRig->getRightVector());
		glm::fquat deltaRotationFront = glmUtils::calcDeltaRotation(deltaRotationUp * originalFrontVector , glm::vec3(0, 1, 0), m_ikRig->getUpVector());
		glm::fquat deltaRotation = deltaRotationFront * deltaRotationUp;
		AnimationClip::AnimationTrack& rootTrack = animation->m_animationTracks[animation->GetTrackIndex(0)];
		for (int i = 0; i < rootTrack.rotations.size(); i++) {
			rootTrack.rotations[i] = deltaRotation * rootTrack.rotations[i];
		}	
		for (int i = 0; i < rootTrack.positions.size(); i++) {
			rootTrack.positions[i] = deltaRotation * rootTrack.positions[i];
		}

	}







}