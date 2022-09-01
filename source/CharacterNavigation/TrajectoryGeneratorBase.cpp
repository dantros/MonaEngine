#include "TrajectoryGeneratorBase.hpp"
#include "IKRig.hpp"

namespace Mona{


EETrajectory::EETrajectory(LIC<3> trajectory, TrajectoryType trajectoryType, int subTrajectoryID) {
        m_curve = trajectory;
        m_trajectoryType = trajectoryType;
        m_subTrajectoryID = subTrajectoryID;
    }

    void HipGlobalTrajectoryData::init(IKRigConfig* config) {
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

    LIC<3> HipGlobalTrajectoryData::sampleOriginalPositions(float initialExtendedAnimTime, float finalExtendedAnimTime) {
		return sampleOriginaCurve(initialExtendedAnimTime, finalExtendedAnimTime, m_originalPositions);
    }

	void HipGlobalTrajectoryData::clear() {
		m_targetPositions = LIC<3>();
		m_motionInitialized = false;
		m_savedPositions = LIC<3>();
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

    void  EEGlobalTrajectoryData::init(IKRigConfig* config, EEGlobalTrajectoryData* oppositeTrData) {
		int frameNum = config->getFrameNum();
		m_savedPositions = LIC<3>();
        m_supportHeights = std::vector<float>(frameNum);
		m_config = config;
		m_oppositeTrajectoryData = oppositeTrData;
    }

    EETrajectory EEGlobalTrajectoryData::getSubTrajectoryByID(int subTrajectoryID) {
		int foundTrIndex = -1;
        for (int i = 0; i < m_originalSubTrajectories.size(); i++) {
            if (m_originalSubTrajectories[i].getSubTrajectoryID() == subTrajectoryID) {
				foundTrIndex = i;
            }
        }
        MONA_ASSERT(foundTrIndex != -1, "EEGlobalTrajectoryData: A sub trajectory with id {0} was not found.", subTrajectoryID);
        return m_originalSubTrajectories[foundTrIndex];
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

	EEGlobalTrajectoryData* EEGlobalTrajectoryData::getOppositeTrajectoryData() {
		return m_oppositeTrajectoryData;
	}

	void EEGlobalTrajectoryData::clear() {
		m_targetTrajectory.m_curve = LIC<3>();
		m_targetTrajectory.m_subTrajectoryID = -1;
		m_fixedTarget = false;
		m_motionInitialized = false;
		m_savedPositions = LIC<3>();
	}


}