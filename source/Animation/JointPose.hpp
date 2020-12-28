#pragma once 
#ifndef JOINTPOSE_HPP
#define JOINTPOSE_HPP
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
namespace Mona {
	struct JointPose {
		glm::fquat m_rotation;
		glm::vec3 m_translation;
		glm::vec3 m_scale;

		JointPose(const glm::fquat& rotation, const glm::vec3& translation, const glm::vec3& scale) :
			m_rotation(rotation), m_translation(translation), m_scale(scale) {}

		JointPose() :
			m_rotation(glm::fquat(1.0f, 0.0f, 0.0f, 0.0f)),
			m_translation(glm::vec3(0.0f)),
			m_scale(glm::vec3(1.0f)) {}
		
		JointPose(const JointPose& rhs) : 
			m_rotation(rhs.m_rotation),
			m_translation(rhs.m_translation),
			m_scale(rhs.m_scale)
		{
		}

		JointPose& operator=(const JointPose& rhs) {
			m_rotation = rhs.m_rotation;
			m_translation = rhs.m_translation;
			m_scale = rhs.m_scale;
			return *this;
		}


	};
	
	inline JointPose operator*(const JointPose& lhs, const JointPose& rhs) {
		JointPose out;
		out.m_scale = lhs.m_scale * rhs.m_scale;
		out.m_rotation = lhs.m_rotation * rhs.m_rotation;
		out.m_translation = lhs.m_rotation * (lhs.m_scale * rhs.m_translation);
		out.m_translation = out.m_translation + lhs.m_translation;
		return out;
	}

	inline JointPose inverse(const JointPose& pose) {
		JointPose inv;
		
		inv.m_scale.x = glm::abs(pose.m_scale.x) < glm::epsilon<float>() ? 0.0f : 1.0f / pose.m_scale.x;
		inv.m_scale.y = glm::abs(pose.m_scale.y) < glm::epsilon<float>() ? 0.0f : 1.0f / pose.m_scale.y;
		inv.m_scale.z = glm::abs(pose.m_scale.z) < glm::epsilon<float>() ? 0.0f : 1.0f / pose.m_scale.z;
		inv.m_rotation = glm::inverse(pose.m_rotation);
		glm::vec3 newTranslation = pose.m_translation * -1.0f;
		inv.m_translation = inv.m_rotation * (inv.m_scale * newTranslation);
		return inv;
	}

	inline JointPose mix(const JointPose& lhs, const JointPose& rhs, float t) {

		return JointPose(glm::slerp(lhs.m_rotation, rhs.m_rotation, t),
			glm::mix(lhs.m_translation, rhs.m_translation, t),
			glm::mix(lhs.m_scale, rhs.m_scale, t));
	}

	inline glm::mat4 JointPoseToMat4(const JointPose& pose) {
		const glm::mat4 translationMatrix = glm::translate(glm::mat4(1.0f), pose.m_translation);
		const glm::mat4 rotationMatrix = glm::toMat4(pose.m_rotation);
		const glm::mat4 scaleMatrix = glm::scale(glm::mat4(1.0f), pose.m_scale);
		return translationMatrix * rotationMatrix * scaleMatrix;
	}
	
}
#endif