#include "TrajectoryGeneratorBase.hpp"
#include "IKRig.hpp"

namespace Mona{


EETrajectory::EETrajectory(LIC<3> trajectory, TrajectoryType trajectoryType, int subTrajectoryID, LIC<3> altCurve) {
        m_curve = trajectory;
        m_trajectoryType = trajectoryType;
        m_subTrajectoryID = subTrajectoryID;
		m_altCurve = altCurve;
    }

    glm::fquat HipGlobalTrajectoryData::getTargetRotation(float reproductionTime) {
        return glm::angleAxis(m_targetRotationAngles.evalCurve(reproductionTime)[0], m_targetRotationAxes.evalCurve(reproductionTime));
    }

    glm::vec3 HipGlobalTrajectoryData::getTargetTranslation(float reproductionTime) {
        return m_targetTranslations.evalCurve(reproductionTime);
    }


    void HipGlobalTrajectoryData::init(IKRigConfig* config) {
		int frameNum = config->getFrameNum();
        m_savedTranslations = std::vector<glm::vec3>(frameNum);
		m_savedDataValid = std::vector<bool>(frameNum, false);
        m_config = config;
    }

	template <int D>
	LIC<D> HipGlobalTrajectoryData::sampleOriginaCurve(float initialExtendedAnimTime, float finalExtendedAnimTime,
		LIC<D>& originalCurve) {
		MONA_ASSERT(initialExtendedAnimTime < finalExtendedAnimTime,
			"HipGlobalTrajectoryData: finalExtendedTime must be greater than initialExtendedTime.");
		LIC<D> extendedCurve = originalCurve;
		while (!(extendedCurve.inTRange(initialExtendedAnimTime) && extendedCurve.inTRange(finalExtendedAnimTime))) {
			extendedCurve = LIC<D>::connect(extendedCurve, originalCurve, true);
			extendedCurve = LIC<D>::connect(extendedCurve, originalCurve, false);
		}
		return extendedCurve.sample(initialExtendedAnimTime, finalExtendedAnimTime);
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
		FrameIndex currFrame = m_config->getFrame(animationTime);
		float nextAnimationTime = m_config->getAnimationTime((currFrame + 1) % m_config->getFrameNum());
		if (nextAnimationTime < animationTime) {
			nextAnimationTime += m_config->getAnimationDuration();
		}
		for (int i = 0; i < m_originalSubTrajectories.size(); i++) {
			if (m_originalSubTrajectories[i].getEECurve().inTRange(animationTime) &&
				m_originalSubTrajectories[i].getEECurve().inTRange(nextAnimationTime)) {
				return m_originalSubTrajectories[i];
			}

		}
		MONA_LOG_ERROR("EETrajectoryData: AnimationTime was not valid.");
		return EETrajectory();
	}

    void  EEGlobalTrajectoryData::init(IKRigConfig* config) {
		int frameNum = config->getFrameNum();
        m_savedPositions = std::vector<glm::vec3>(frameNum);
		m_savedDataValid = std::vector<bool>(frameNum, false);
        m_supportHeights = std::vector<float>(frameNum);
		m_config = config;
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
			extendedCurve = LIC<3>::connect(extendedCurve, additionalCurve);
		}
		return extendedCurve.sample(animationTime, animationTime + duration);
	}


}