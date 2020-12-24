#pragma once
#ifndef SKELETON_HPP
#define SKELETON_HPP
#include <cstdint>
#include <string>
#include <vector>
#include <unordered_map>
#include <string_view>
#include <glm/glm.hpp>
namespace Mona {


	class Skeleton {
	public:
		using size_type = std::vector<std::string>::size_type;
		Skeleton() : m_invBindPoseMatrices(), m_jointNames(), m_parentIndices() {}

		Skeleton(std::vector<glm::mat4>&& invBindPoses,
			std::vector<std::string>&& jointNames,
			std::vector<std::int32_t>&& parentIndices) :
			m_invBindPoseMatrices(std::move(invBindPoses)),
			m_jointNames(std::move(jointNames)),
			m_parentIndices(std::move(parentIndices))
		{
			BuildJointMap();
		}

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
		void BuildJointMap() {
			m_jointMap.reserve(m_jointNames.size());
			for (uint32_t i = 0; i < m_jointNames.size(); i++) {
				m_jointMap.insert(std::make_pair(m_jointNames[i], i));
			}
		}
		std::unordered_map<std::string, uint32_t> m_jointMap;
		std::vector<glm::mat4> m_invBindPoseMatrices;
		std::vector<std::string> m_jointNames;
		std::vector<std::int32_t> m_parentIndices;
	};
}
#endif