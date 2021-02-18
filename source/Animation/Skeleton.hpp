#pragma once
#ifndef SKELETON_HPP
#define SKELETON_HPP
#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>
#include <glm/glm.hpp>
namespace Mona {


	class Skeleton {
	public:
		friend class SkeletonManager;
		using size_type = std::vector<std::string>::size_type;
		size_type JointCount() const {
			return m_jointNames.size();
		}
		const std::string& GetJointName(size_type index) const {
			return m_jointNames[index];
		}

		const std::vector<glm::mat4>& GetInverseBindPoseMatrices() const
		{ 
			return m_invBindPoseMatrices; 
		}

		const glm::mat4& GetInverseBindPoseMatrix(uint32_t jointIndex) const {
			return m_invBindPoseMatrices[jointIndex];
		}

		std::int32_t GetJointIndex(const std::string& name) const {
			auto it = m_jointMap.find(name);
			if (it != m_jointMap.end()) {
				return (*it).second;
			}

			return -1;
		}

		std::int32_t GetParentIndex(size_type index) const {
			return m_parentIndices[index];
		}
	private:
		//Skeleton() : m_invBindPoseMatrices(), m_jointNames(), m_parentIndices() {}
		/*
		Skeleton(std::vector<glm::mat4>&& invBindPoses,
			std::vector<std::string>&& jointNames,
			std::vector<std::int32_t>&& parentIndices,
			std::unordered_map<std::string, uint32_t> &&jointMap) :
			m_invBindPoseMatrices(std::move(invBindPoses)),
			m_jointNames(std::move(jointNames)),
			m_parentIndices(std::move(parentIndices)),
			m_jointMap(std::move(jointMap))
		{
		}
		*/

		Skeleton(const std::string &filePath);
		std::unordered_map<std::string, uint32_t> m_jointMap;
		std::vector<glm::mat4> m_invBindPoseMatrices;
		std::vector<std::string> m_jointNames;
		std::vector<std::int32_t> m_parentIndices;
	};
}
#endif