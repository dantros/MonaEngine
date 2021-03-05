#include "AnimationController.hpp"
#include "AnimationClip.hpp"
#include "Skeleton.hpp"
#include <glm/gtx/matrix_decompose.hpp>
#include "../Core/Log.hpp"
namespace Mona {
	void BlendPoses(std::vector<JointPose>& output,
		std::vector<JointPose>& firstPose,
		std::vector<JointPose>& secondPose,
		float t)
	{
		for (uint32_t i = 0; i < output.size(); i++) {
			output[i] = mix(firstPose[i], secondPose[i], t);
		}
	}

	AnimationController::AnimationController(std::shared_ptr<AnimationClip> animation) noexcept : m_animationClipPtr(animation)
	{
		
		MONA_ASSERT(animation != nullptr, "AnimationController Error: Starting animation cannot be null.");
		auto skeleton = animation->GetSkeleton();
		m_currentPose.resize(skeleton->JointCount());
	}
	void AnimationController::PlayAnimation(std::shared_ptr<AnimationClip> animation) noexcept {
		if (animation == nullptr || m_animationClipPtr == animation)
			return;
		//Al reproducir una animacion nueva, limpianos el reloj actual y el target de animacion
		m_sampleTime = 0.0f;
		m_animationClipPtr = animation;
		m_crossfadeTarget.Clear();
	}

	void AnimationController::FadeTo(std::shared_ptr<AnimationClip> animation,
		BlendType blendType,
		float fadeDuration,
		float startTime) noexcept {

		//Retorno prematuro en el caso de que la nueva animacion objetivo es la misma que el target/objetivo actual o nula.
		if (m_animationClipPtr == animation || m_crossfadeTarget.GetAnimationClip() == animation || animation == nullptr)
			return;
		//Si la animacion objetivo no es nula, esta pasa a ser la principal
		if (!m_crossfadeTarget.IsNullTarget()) {
			m_animationClipPtr = m_crossfadeTarget.m_targetClip;
			m_sampleTime = m_crossfadeTarget.m_sampleTime;
			m_isLooping = m_crossfadeTarget.m_isLooping;
			m_crossfadeTarget.Clear();
		}

		//Se crea el nuevo objetivo a partir de la animacion entregada
		//En caso de que el blendType sea KeepSynchronize entonces se ignorara startTime y se
		// partira con la animacion objetivo sincronizada con la animacion actual.
		float clipDuration = m_animationClipPtr->GetDuration();
		float normalizedTimeClip = m_sampleTime / clipDuration;
		m_crossfadeTarget.SetAnimationClip(animation,
			blendType,
			fadeDuration,
			startTime,
			normalizedTimeClip);
	}

	void AnimationController::ClearFadeTo() noexcept {
		if (m_crossfadeTarget.IsNullTarget())
			return;
		m_crossfadeTarget.Clear();
	}

	void AnimationController::UpdateCurrentPose(float timeStep) noexcept {
		//Se avanza el tiempo pasado en la animación objetivo
		//Si ya ha transcurrido el tiempo dado de reproduccion de la animacion objetivo, esta pasa a ser la principal.
		if (!m_crossfadeTarget.IsNullTarget()) {
			m_crossfadeTarget.m_elapsedTime += timeStep;
			if (m_crossfadeTarget.m_fadeDuration <= m_crossfadeTarget.m_elapsedTime) {
				m_animationClipPtr = m_crossfadeTarget.m_targetClip;
				m_sampleTime = m_crossfadeTarget.m_sampleTime;
				m_isLooping = m_crossfadeTarget.m_isLooping;
				m_crossfadeTarget.Clear();

			}
		}

		//Limpiamos la pose actual
		std::fill(m_currentPose.begin(), m_currentPose.end(), JointPose());

		//Muestreamos los clips de animacion obteniendo las poses en espacio local
		if (!m_crossfadeTarget.IsNullTarget())
		{
			float clipDuration = m_animationClipPtr->GetDuration();
			float playbackFactorClip = 1.0f;
			float playbackFactorTarget = 1.0f;
			float factor = m_crossfadeTarget.m_elapsedTime / m_crossfadeTarget.m_fadeDuration;
			if (factor > 1.0f) factor = 1.0f;
			//Si el tipo de blending es KeepSynchronize se debe hacer el trabajo extra
			// de mantener las animaciones sincronizadas y de interpolar tambien
			// el playback de estas.
			if (m_crossfadeTarget.m_blendType == BlendType::KeepSynchronize)
			{
				float targetDuration = m_crossfadeTarget.m_targetClip->GetDuration();
				float durationRatio = targetDuration / clipDuration;
				playbackFactorTarget = (1.0f - factor) * durationRatio + factor;
				playbackFactorClip = playbackFactorTarget / durationRatio;
			}
			else if (m_crossfadeTarget.m_blendType == BlendType::Freeze) {
				playbackFactorClip = 0.0f;
			}

			//Muestreo de la animación principal
			m_sampleTime = m_animationClipPtr->Sample(m_currentPose,
				m_sampleTime + timeStep * m_playRate * playbackFactorClip,
				m_isLooping);

			auto& targetPose = m_crossfadeTarget.m_currentPose;
			std::fill(targetPose.begin(), targetPose.end(), JointPose());
			//Muestreo de la animación objetivo
			m_crossfadeTarget.m_sampleTime = m_crossfadeTarget.m_targetClip->Sample(targetPose,
				m_crossfadeTarget.m_sampleTime + timeStep * m_playRate * playbackFactorTarget,
				m_crossfadeTarget.m_isLooping);
			//Interpolacion entre ambas poses
			BlendPoses(m_currentPose, m_currentPose, targetPose, factor);
		}
		else
		{
			m_sampleTime = m_animationClipPtr->Sample(m_currentPose, m_sampleTime + timeStep * m_playRate, m_isLooping);
		}


		//Recorrido del esqueleto para pasar de poses en espacio local a global.
		auto skeleton = m_animationClipPtr->GetSkeleton();
		for (uint32_t i = 1; i < m_currentPose.size(); i++) {
			int32_t parentIndex = skeleton->GetParentIndex(i);
			m_currentPose[i] = m_currentPose[parentIndex] * m_currentPose[i];
		}
	}

	void AnimationController::GetMatrixPalette(std::vector<glm::mat4>& outMatrixPalette) const
	{
		
		auto skeleton = m_animationClipPtr->GetSkeleton();
		auto& invBindPoseMatrices = skeleton->GetInverseBindPoseMatrices();
		//Se expresan las poses como matrices y se multiplican por la inverse bind pose antes de enviar
		// la información al renderer.
		for (uint32_t i = 0; i < skeleton->JointCount(); i++) {
			outMatrixPalette[i] = JointPoseToMat4(m_currentPose[i]) * invBindPoseMatrices[i];
		}
	}

	JointPose AnimationController::GetJointModelPose(uint32_t jointIndex) const {
		auto skeleton = m_animationClipPtr->GetSkeleton();
		const glm::mat4& invBindMatrix = skeleton->GetInverseBindPoseMatrix(jointIndex); 
		glm::vec3 scale;
		glm::fquat rotation;
		glm::vec3 translation;
		glm::vec3 skew;
		glm::vec4 perspective;
		glm::decompose(invBindMatrix, scale, rotation, translation, skew, perspective);
		return m_currentPose[jointIndex] * JointPose(rotation, translation, scale);
	}

}