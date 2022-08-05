#include "TrajectoryGeneratorBase.hpp"
#include "IKRig.hpp"

namespace Mona{


EETrajectory::EETrajectory(LIC<3> trajectory, TrajectoryType trajectoryType, int subTrajectoryID) {
        m_curve = trajectory;
        m_trajectoryType = trajectoryType;
        m_subTrajectoryID = subTrajectoryID;
    }

    glm::fquat HipGlobalTrajectoryData::getTargetRotation(float reproductionTime) {
        return glm::angleAxis(m_targetRotationAngles.evalCurve(reproductionTime)[0], m_targetRotationAxes.evalCurve(reproductionTime));
    }

    glm::vec3 HipGlobalTrajectoryData::getTargetTranslation(float reproductionTime) {
        return m_targetTranslations.evalCurve(reproductionTime);
    }


    void HipGlobalTrajectoryData::init(int frameNum, IKRigConfig* config) {
        m_savedRotationAngles = std::vector<float>(frameNum);
        m_savedRotationAxes = std::vector<glm::vec3>(frameNum);
        m_savedTranslations = std::vector<glm::vec3>(frameNum);
        m_config = config;
    }

    LIC<1> HipGlobalTrajectoryData::sampleOriginalRotationAngles(float initialExtendedAnimTime, float finalExtendedAnimTime) {
		MONA_ASSERT((finalExtendedAnimTime - initialExtendedAnimTime) <= m_config->getAnimationDuration(),
			"HipGlobalTrajectoryData: input extended times were invalid.");
		MONA_ASSERT(m_originalRotationAngles.inTRange(initialExtendedAnimTime) || m_originalRotationAngles.inTRange(finalExtendedAnimTime),
			"HipGlobalTrajectoryData: input extended times were invalid.");
		MONA_ASSERT(initialExtendedAnimTime < finalExtendedAnimTime,
			"HipGlobalTrajectoryData: finalExtendedTime must be greater than initialExtendedTime.");
		float initialAnimTime = m_config->adjustAnimationTime(initialExtendedAnimTime);
		float finalAnimTime = m_config->adjustAnimationTime(finalExtendedAnimTime);
		if (initialAnimTime < finalAnimTime) {
			return m_originalRotationAngles.sample(initialAnimTime, finalAnimTime);
		}
		else {
			LIC<1> part1 = m_originalRotationAngles.sample(initialAnimTime, m_originalRotationAngles.getTRange()[1]);
			LIC<1> part2 = m_originalRotationAngles.sample(m_originalRotationAngles.getTRange()[0], finalAnimTime);
			LIC<1> result = LIC<1>::connect(part1, part2, m_config->getAnimationDuration());
			result.offsetTValues(-result.getTRange()[0]);
			result.offsetTValues(initialExtendedAnimTime);
			return result;
		}
    }
    LIC<3> HipGlobalTrajectoryData::sampleOriginalRotationAxes(float initialExtendedAnimTime, float finalExtendedAnimTime) {
		MONA_ASSERT((finalExtendedAnimTime - initialExtendedAnimTime) <= m_config->getAnimationDuration(),
			"HipGlobalTrajectoryData: input extended times were invalid.");
		MONA_ASSERT(m_originalRotationAxes.inTRange(initialExtendedAnimTime) || m_originalRotationAxes.inTRange(finalExtendedAnimTime),
			"HipGlobalTrajectoryData: input extended times were invalid.");
		MONA_ASSERT(initialExtendedAnimTime < finalExtendedAnimTime,
			"HipGlobalTrajectoryData: finalExtendedTime must be greater than initialExtendedTime.");
		float initialAnimTime = m_config->adjustAnimationTime(initialExtendedAnimTime);
		float finalAnimTime = m_config->adjustAnimationTime(finalExtendedAnimTime);
		if (initialAnimTime < finalAnimTime) {
			return m_originalRotationAxes.sample(initialAnimTime, finalAnimTime);
		}
		else {
			LIC<3> part1 = m_originalRotationAxes.sample(initialAnimTime, m_originalRotationAxes.getTRange()[1]);
			LIC<3> part2 = m_originalRotationAxes.sample(m_originalRotationAxes.getTRange()[0], finalAnimTime);
			LIC<3> result = LIC<3>::connect(part1, part2, m_config->getAnimationDuration());
			result.offsetTValues(-result.getTRange()[0]);
			result.offsetTValues(initialExtendedAnimTime);
			return result;
		}
    }
    LIC<3> HipGlobalTrajectoryData::sampleOriginalTranslations(float initialExtendedAnimTime, float finalExtendedAnimTime) {
		float epsilonAnimDuration = m_config->getAnimationDuration();
		funcUtils::epsilonAdjustment_add(epsilonAnimDuration, 0.000001);
		MONA_ASSERT((finalExtendedAnimTime - initialExtendedAnimTime) <= epsilonAnimDuration,
			"HipGlobalTrajectoryData: input extended times were invalid.");
        MONA_ASSERT(m_originalTranslations.inTRange(initialExtendedAnimTime) || m_originalTranslations.inTRange(finalExtendedAnimTime),
            "HipGlobalTrajectoryData: input extended times were invalid.");
        MONA_ASSERT(initialExtendedAnimTime < finalExtendedAnimTime, 
            "HipGlobalTrajectoryData: finalExtendedTime must be greater than initialExtendedTime.");
        float initialAnimTime = m_config->adjustAnimationTime(initialExtendedAnimTime);
        float finalAnimTime = m_config->adjustAnimationTime(finalExtendedAnimTime);
		if (initialAnimTime < finalAnimTime) {
			return m_originalTranslations.sample(initialAnimTime, finalAnimTime);
		}
        else {
			LIC<3> part1 = m_originalTranslations.sample(initialAnimTime, m_originalTranslations.getTRange()[1]);
			LIC<3> part2 = m_originalTranslations.sample(m_originalTranslations.getTRange()[0], finalAnimTime);
			LIC<3> result = LIC<3>::connect(part1, part2, m_config->getAnimationDuration());
			result.offsetTValues(-result.getTRange()[0]);
            result.offsetTValues(initialExtendedAnimTime);
            return result;
        }
    }

	EETrajectory EEGlobalTrajectoryData::getSubTrajectory(float animationTime) {
		for (int i = 0; i < m_originalSubTrajectories.size(); i++) {
			if (i == m_originalSubTrajectories.size() - 1) {
				if (m_originalSubTrajectories[i].getEECurve().inTRange(animationTime)) {
					return m_originalSubTrajectories[i];
				}
			}
			else {
				if (m_originalSubTrajectories[i].getEECurve().inTRange(animationTime) &&
					m_originalSubTrajectories[i + 1].getEECurve().inTRange(animationTime)) {
					return m_originalSubTrajectories[i + 1];
				}
				else if (m_originalSubTrajectories[i].getEECurve().inTRange(animationTime)) {
					return m_originalSubTrajectories[i];
				}
			}

		}
		MONA_LOG_ERROR("EETrajectoryData: AnimationTime was not valid.");
		return EETrajectory();
	}

    void  EEGlobalTrajectoryData::init(int frameNum) {
        m_savedPositions = std::vector<glm::vec3>(frameNum);
        m_supportHeights = std::vector<float>(frameNum);
    }

    EETrajectory EEGlobalTrajectoryData::getSubTrajectoryByID(int subTrajectoryID) {
        for (int i = 0; i < m_originalSubTrajectories.size(); i++) {
            if (m_originalSubTrajectories[i].getSubTrajectoryID() == subTrajectoryID) {
                return m_originalSubTrajectories[i];
            }
        }
        MONA_LOG_ERROR("EEGlobalTrajectoryData: A sub trajectory with id {0} was not found.", subTrajectoryID);
        return EETrajectory();
    }


}