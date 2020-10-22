#pragma once
#ifndef AUDIOSOURCECOMPONENTLIFETIMEPOLICY_HPP
#define AUDIOSOURCECOMPONENTLIFETIMEPOLICY_HPP
#include "AudioSystem.hpp"
#include "AudioSourceComponent.hpp"
namespace Mona {
	class GameObject;
	class AudioSourceComponentLifetimePolicy {
	public:
		AudioSourceComponentLifetimePolicy() = default;
		AudioSourceComponentLifetimePolicy(AudioSystem* audioSystem) : m_audioSystem(audioSystem) {}
		void OnAddComponent(GameObject* gameObjectPtr, AudioSourceComponent& audioSource, const InnerComponentHandle& handle) {
			audioSource.SetTransformHandle(gameObjectPtr->GetInnerComponentHandle<TransformComponent>());
		}
		void OnRemoveComponent(GameObject* gameObjectPtr,AudioSourceComponent& audioSource, const InnerComponentHandle& handle) {
			if (audioSource.m_openALsource) {
				const auto& alSource = audioSource.m_openALsource.value();
				m_audioSystem->RemoveOpenALSource(alSource.m_sourceIndex);

			}
		}

	private:
		AudioSystem* m_audioSystem;
	};
}
#endif