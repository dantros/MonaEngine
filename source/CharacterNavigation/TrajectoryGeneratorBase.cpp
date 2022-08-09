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

	template <int D>
	LIC<D> HipGlobalTrajectoryData::sampleOriginaCurve(float initialExtendedAnimTime, float finalExtendedAnimTime,
		LIC<D>& originalCurve) {
		float epsilonAnimDuration = m_config->getAnimationDuration();
		funcUtils::epsilonAdjustment_add(epsilonAnimDuration, originalCurve.getTEpsilon());
		MONA_ASSERT((finalExtendedAnimTime - initialExtendedAnimTime) <= epsilonAnimDuration,
			"HipGlobalTrajectoryData: input extended times were invalid.");
		MONA_ASSERT(initialExtendedAnimTime < finalExtendedAnimTime,
			"HipGlobalTrajectoryData: finalExtendedTime must be greater than initialExtendedTime.");
		float initialAnimTime = m_config->adjustAnimationTime(initialExtendedAnimTime);
		float finalAnimTime = m_config->adjustAnimationTime(finalExtendedAnimTime);
		if (initialAnimTime < finalAnimTime) {
			return originalCurve.sample(initialAnimTime, finalAnimTime);
		}
		else {
			LIC<D> part1 = originalCurve.sample(initialAnimTime, originalCurve.getTRange()[1]);
			LIC<D> part2 = originalCurve.sample(originalCurve.getTRange()[0], finalAnimTime);
			float tDiff = part2.getTRange()[0] + m_config->getAnimationDuration() - part1.getTRange()[1];
			LIC<D> result = LIC<D>::connect(part1, part2, tDiff);
			result.offsetTValues(-result.getTRange()[0]);
			result.offsetTValues(initialExtendedAnimTime);
			return result;
		}
	}

    LIC<1> HipGlobalTrajectoryData::sampleOriginalRotationAngles(float initialExtendedAnimTime, float finalExtendedAnimTime) {
		return sampleOriginaCurve(initialExtendedAnimTime, finalExtendedAnimTime, m_originalRotationAngles);
    }
    LIC<3> HipGlobalTrajectoryData::sampleOriginalRotationAxes(float initialExtendedAnimTime, float finalExtendedAnimTime) {
		return sampleOriginaCurve(initialExtendedAnimTime, finalExtendedAnimTime, m_originalRotationAxes);
    }
    LIC<3> HipGlobalTrajectoryData::sampleOriginalTranslations(float initialExtendedAnimTime, float finalExtendedAnimTime) {
		return sampleOriginaCurve(initialExtendedAnimTime, finalExtendedAnimTime, m_originalTranslations);
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

	LIC<3> EEGlobalTrajectoryData::sampleExtendedSubTrajectory(float animationTime, float duration) {
		EETrajectory firstTr = getSubTrajectory(animationTime);
		LIC<3> extendedCurve = firstTr.getEECurve();
		int currID = firstTr.getSubTrajectoryID();
		while (!extendedCurve.inTRange(animationTime + duration)) {
			currID = (currID + 1) % m_originalSubTrajectories.size();
			EETrajectory additionalTr = getSubTrajectoryByID(currID);
			LIC<3> additionalCurve = additionalTr.getEECurve();
			extendedCurve = LIC<3>::connect(extendedCurve, additionalCurve, 0);
		}
		return extendedCurve.sample(animationTime, animationTime + duration);
	}


}