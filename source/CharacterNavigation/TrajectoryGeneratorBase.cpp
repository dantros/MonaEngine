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


	// primer termino: acercar los modulos de las velocidades
	std::function<float(const std::vector<float>&, TGData*)> term1Function =
		[](const std::vector<float>& varPCoord, TGData* dataPtr)->float {
		float result = 0;
		for (int i = 0; i < dataPtr->pointIndexes.size(); i++) {
			int pIndex = dataPtr->pointIndexes[i];
			result += glm::distance2(dataPtr->varCurve->getPointVelocity(pIndex), dataPtr->baseCurve.getPointVelocity(pIndex));
			result += glm::distance2(dataPtr->varCurve->getPointVelocity(pIndex, true), dataPtr->baseCurve.getPointVelocity(pIndex, true));
		}
		return result;
	};

	std::function<float(const std::vector<float>&, int, TGData*)> term1PartialDerivativeFunction =
		[](const std::vector<float>& varPCoord, int varIndex, TGData* dataPtr)->float {
		int D = 3;
		int pIndex = dataPtr->pointIndexes[varIndex / D];
		int coordIndex = varIndex % D;
		float t_kPrev = dataPtr->varCurve->getTValue(pIndex - 1);
		float t_kCurr = dataPtr->varCurve->getTValue(pIndex);
		float t_kNext = dataPtr->varCurve->getTValue(pIndex + 1);
		glm::vec3 lVel = dataPtr->varCurve->getPointVelocity(pIndex);
		glm::vec3 rVel = dataPtr->varCurve->getPointVelocity(pIndex, true);
		glm::vec3 baseLVel = dataPtr->baseCurve.getPointVelocity(pIndex);
		glm::vec3 baseRVel = dataPtr->baseCurve.getPointVelocity(pIndex, true);
		float result = 0;
		result += 2 * (lVel[coordIndex] - baseLVel[coordIndex]) * (1 / (t_kCurr - t_kPrev));
		result += 2 * (rVel[coordIndex] - baseRVel[coordIndex]) * (-1 / (t_kNext - t_kCurr));
		return result;
	};

	std::function<void(std::vector<float>&, TGData*, std::vector<float>&, int)>  postDescentStepCustomBehaviour =
		[](std::vector<float>& varPCoord, TGData* dataPtr, std::vector<float>& argsRawDelta, int varIndex_progressive)->void {
		glm::vec3 newPos;
		int D = 3;
		for (int i = 0; i < dataPtr->pointIndexes.size(); i++) {
			for (int j = 0; j < D; j++) {
				if (varPCoord[i * D + j] <= dataPtr->minValues[i * D + j]) {
					varPCoord[i * D + j] = dataPtr->minValues[i * D + j];
					argsRawDelta[i * D + j] *= 0.3f;
				}
				newPos[j] = varPCoord[i * D + j];
			}
			int pIndex = dataPtr->pointIndexes[i];
			dataPtr->varCurve->setCurvePoint(pIndex, newPos);
		}
	};


	void StrideCorrector::init() {
		FunctionTerm<TGData> term1(term1Function, term1PartialDerivativeFunction);
		m_gradientDescent = GradientDescent<TGData>({ term1 }, 0, &m_tgData, postDescentStepCustomBehaviour);
		m_tgData.descentRate = 1 / pow(10, 3);
		m_tgData.maxIterations = 600;
		m_tgData.targetPosDelta = 1 / pow(10, 6);
		m_gradientDescent.setTermWeight(0, 1.0f);
	}

	void StrideCorrector::correctStride(LIC<3>& targetCurve, LIC<3>& originalCurve,
		EnvironmentData& environmentData,
		ComponentManager<TransformComponent>& transformManager,
		ComponentManager<StaticMeshComponent>& staticMeshManager) {
		if (targetCurve.getNumberOfPoints() == 2) {
			return;
		}
		float startSupportHeight = originalCurve.getStart()[2];
		float endSupportHeight = originalCurve.getEnd()[2];

		LIC<3> baseCurve = targetCurve;

		// reajuste de altura del final de la curva para corregir posibles errores de posicionamiento
		// se guarda la curva sin modificar en baseCurve, ya que es importante preservar su forma
		glm::vec2 xyEndPoint = targetCurve.getEnd();
		float terrainHeightEndPoint = environmentData.getTerrainHeight(xyEndPoint, transformManager, staticMeshManager);
		glm::vec3 adjustedEndPoint = glm::vec3(xyEndPoint, terrainHeightEndPoint + endSupportHeight);
		targetCurve.setCurvePoint(targetCurve.getNumberOfPoints() - 1, adjustedEndPoint);


		m_tgData.pointIndexes.clear();
		m_tgData.minValues.clear();
		for (int i = 1; i < targetCurve.getNumberOfPoints() - 1; i++) {
			m_tgData.pointIndexes.push_back(i);
			glm::vec3 currPoint = targetCurve.getCurvePoint(i);
			float fraction = funcUtils::getFraction(0, targetCurve.getNumberOfPoints() - 1, i);
			float currSupportHeight = funcUtils::lerp(startSupportHeight, endSupportHeight, fraction);
			float minZ = environmentData.getTerrainHeight(glm::vec2(currPoint), transformManager, staticMeshManager) + currSupportHeight;
			m_tgData.minValues.push_back(std::numeric_limits<float>::lowest());
			m_tgData.minValues.push_back(std::numeric_limits<float>::lowest());
			m_tgData.minValues.push_back(minZ);
		}

		// valores iniciales y curva base
		std::vector<float> initialArgs(m_tgData.pointIndexes.size() * 3);
		for (int i = 0; i < m_tgData.pointIndexes.size(); i++) {
			int pIndex = m_tgData.pointIndexes[i];
			for (int j = 0; j < 3; j++) {
				initialArgs[i * 3 + j] = targetCurve.getCurvePoint(pIndex)[j];
			}
		}
		m_tgData.baseCurve = baseCurve;
		m_tgData.varCurve = &targetCurve;

		m_gradientDescent.setArgNum(initialArgs.size());
		m_gradientDescent.computeArgsMin(m_tgData.descentRate, m_tgData.maxIterations, m_tgData.targetPosDelta, initialArgs);
	}


}