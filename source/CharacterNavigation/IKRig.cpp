#include "IKRig.hpp"
#include "../Utilities/FuncUtils.hpp"


namespace Mona {

	std::vector<MatrixXf> IKRig::getClipRotations(std::shared_ptr<AnimationClip> clip, int frame) {
		std::vector<glm::vec3> positions = std::vector<glm::vec3>(m_bvhData.getJointNum());
		std::vector<glm::fquat> rotations = std::vector<glm::fquat>(m_bvhData.getJointNum());
		std::vector<glm::vec3> scales = std::vector<glm::vec3>(m_bvhData.getJointNum());
		for (int i = 0; i < clip->m_animationTracks.size(); i++) {
            int jointIndex = funcUtils::findIndex<std::string>(m_bvhData.getJointNames(), clip->m_trackJointNames[i]);
            if (jointIndex == -1) {
                continue;
            }
			positions[jointIndex] = clip->m_animationTracks[i].positions[frame];
			rotations[jointIndex] = clip->m_animationTracks[i].positions[frame];
			scales[jointIndex] = clip->m_animationTracks[i].positions[frame];
		}
		return std::vector<MatrixXf>(0);
	}



}
