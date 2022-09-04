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
		for (int i = 0; i < m_ikRig->m_animationConfigs.size(); i++) {
			if (m_ikRig->m_animationConfigs[i].m_animationClip->GetAnimationName() == animationClip->GetAnimationName()) {
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

	void AnimationValidator::checkLegsRotationAxes_base(IKRigConfig* config) {
		std::shared_ptr<AnimationClip> anim = config->m_animationClip;
		std::vector<JointIndex>const& topology = m_ikRig->getTopology();
		for (ChainIndex c = 0; c < m_ikRig->getChainNum(); c++) {
			IKChain chain = m_ikRig->m_ikChains[c];
			for (int i = 0; i < chain.getJoints().size() - 1; i++) {
				std::vector<glm::vec3> globalRotationAxes;
				JointIndex jIndex = chain.getJoints()[i];
				for (FrameIndex j = 0; j < config->getFrameNum(); j++) {
					auto baseRotations = config->getBaseJointRotations(j);
					glm::fquat globalRotation = baseRotations[jIndex].getQuatRotation();
					JointIndex parent = topology[jIndex];
					while (parent != -1) {
						globalRotation = baseRotations[parent].getQuatRotation() * globalRotation;
						parent = topology[parent];
					}
					globalRotationAxes.push_back(glm::axis(globalRotation));
				}

				// buscamos el eje de rotacion global principal
				glm::vec3 meanRotationAxis(0);
				for (FrameIndex j = 0; j < config->getFrameNum(); j++) {
					meanRotationAxis += globalRotationAxes[j];

				}
				meanRotationAxis /= config->getFrameNum();

				MONA_ASSERT(0.96f < abs(meanRotationAxis[0]), "AnimationValidator: Leg joints must have an x main rotation axis globally.");
				MONA_ASSERT(abs(meanRotationAxis[1]) < 0.3f, "AnimationValidator: Leg joints must have an x main rotation axis globally.");
				MONA_ASSERT(abs(meanRotationAxis[2]) < 0.3f, "AnimationValidator: Leg joints must have an x main rotation axis globally.");

			}
		}
	}


	void AnimationValidator::checkLegGlobalRotationAxes(std::shared_ptr<AnimationClip> animation, IKChain* legChain, glm::fquat baseRotation) {
		std::vector<JointIndex>const& topology = m_ikRig->getTopology();
		int frameNum = animation->m_animationTracks[0].rotations.size();
		for (int i = 0; i < legChain->getJoints().size() - 1; i++) {
			std::vector<glm::vec3> globalRotationAxes;
			JointIndex jIndex = legChain->getJoints()[i];
			for (FrameIndex j = 0; j < frameNum; j++) {
				glm::fquat globalRotation = animation->m_animationTracks[animation->GetTrackIndex(jIndex)].rotations[j];
				JointIndex parent = topology[jIndex];
				while (parent != -1) {
					globalRotation = animation->m_animationTracks[animation->GetTrackIndex(parent)].rotations[j] * globalRotation;
					parent = topology[parent];
				}
				globalRotation = baseRotation * globalRotation;
				globalRotationAxes.push_back(glm::axis(globalRotation));
			}

			// buscamos el eje de rotacion global principal
			glm::vec3 meanRotationAxis(0);
			for (FrameIndex j = 0; j < frameNum; j++) {
				meanRotationAxis += globalRotationAxes[j];

			}
			meanRotationAxis /= frameNum;

			MONA_ASSERT(0.8f < abs(meanRotationAxis[0]), "AnimationValidator: Leg joints must have an x main rotation axis globally.");
			MONA_ASSERT(abs(meanRotationAxis[1]) < 0.3f, "AnimationValidator: Leg joints must have an x main rotation axis globally.");
			MONA_ASSERT(abs(meanRotationAxis[2]) < 0.3f, "AnimationValidator: Leg joints must have an x main rotation axis globally.");

		}
	}



	void AnimationValidator::correctLegLocalRotationAxes(std::shared_ptr<AnimationClip> animation, IKChain* legChain) {
		// modificamos las traslaciones de las articulaciones importantes para que sean del tipo {0,0,z}
		// luego modificamos la rotacion de la articulacion padre para tomar en cuenta ese cambio
		// ajustamos la rotacion/traslacion de la articulacion y sus hermanas para no propagar cambios indeseados
		for (int i = 1; i < legChain->getJoints().size(); i++) {
			JointIndex jIndex = legChain->getJoints()[i];
			JointIndex parent = m_ikRig->getTopology()[jIndex];
			std::vector<JointIndex> levelJoints = m_ikRig->getJointChildren(parent);
			AnimationClip::AnimationTrack& jointTrack = animation->m_animationTracks[animation->GetTrackIndex(jIndex)];
			AnimationClip::AnimationTrack& parentTrack = animation->m_animationTracks[animation->GetTrackIndex(parent)];
			glm::vec3 currentTranslation = jointTrack.positions[0];
			glm::vec3 targetTranslation = glm::vec3(0, 0, glm::length(currentTranslation));
			glm::fquat deltaRotation = glmUtils::calcDeltaRotation(glm::normalize(currentTranslation), glm::normalize(targetTranslation), 
				m_ikRig->getRightVector());
			for (int j = 0; j < levelJoints.size(); j++) {
				AnimationClip::AnimationTrack& levelJointTrack = animation->m_animationTracks[animation->GetTrackIndex(levelJoints[j])];
				for (int k = 0; k < levelJointTrack.positions.size(); k++) {
					levelJointTrack.positions[k] = deltaRotation * levelJointTrack.positions[k];
				}
				for (int k = 0; k < levelJointTrack.rotations.size(); k++) {
					levelJointTrack.rotations[k] = deltaRotation * levelJointTrack.rotations[k];
				}
			}
			for (int j = 0; j < parentTrack.rotations.size(); j++) {
				parentTrack.rotations[j] = parentTrack.rotations[j] * glm::inverse(deltaRotation);				
			}
		}

	}


	void AnimationValidator::debugRotationAxes(IKRigConfig* config, std::vector<JointIndex> targetJoints) {
		std::shared_ptr<AnimationClip> anim = config->m_animationClip;
		std::vector<JointIndex>const& topology = m_ikRig->getTopology();
		for (int i = 0; i < targetJoints.size(); i++) {
			std::vector<glm::vec3> globalRotationAxes;
			std::vector<float> globalRotationAngles;
			std::vector<glm::vec3> localRotationAxes;
			std::vector<float> localRotationAngles;
			JointIndex jIndex = targetJoints[i];
			for (FrameIndex j = 0; j < config->getFrameNum(); j++) {
				auto baseRotations = config->getBaseJointRotations(j);
				glm::fquat globalRotation = baseRotations[jIndex].getQuatRotation();
				localRotationAxes.push_back(glm::axis(globalRotation));;
				localRotationAngles.push_back(glm::angle(globalRotation));
				JointIndex parent = topology[jIndex];
				while (parent != -1) {
					JointRotation parentRot = baseRotations[parent];
					globalRotation = parentRot.getQuatRotation() * globalRotation;
					parent = topology[parent];
				}
				globalRotationAxes.push_back(glm::axis(globalRotation));
				globalRotationAngles.push_back(glm::angle(globalRotation));
			}
			std::cout << "JOINT INDEX: " << jIndex << std::endl;
			std::cout << "local rotation angles: " << std::endl;
			std::cout << funcUtils::vecToString(localRotationAngles) << std::endl;
			std::cout << "local rotation axes: " << std::endl;
			glmUtils::printColoredStdVector(localRotationAxes);
			// buscamos el eje de rotacion global principal
			glm::vec3 meanRotationAxis(0);
			for (FrameIndex j = 0; j < config->getFrameNum(); j++) {
				meanRotationAxis += globalRotationAxes[j];

			}
			meanRotationAxis /= config->getFrameNum();
		}

	}










}