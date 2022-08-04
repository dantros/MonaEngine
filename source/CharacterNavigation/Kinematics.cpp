#include "Kinematics.hpp"
#include "../Core/GlmUtils.hpp"
#include "../Core/FuncUtils.hpp"
#include <glm/gtx/matrix_decompose.hpp>
#include "IKRig.hpp"


namespace Mona {

	ForwardKinematics::ForwardKinematics(IKRig* ikRig) {
		m_ikRig = ikRig;
	}

	glm::mat4 rotationMatrixDerivative_dAngle(float angle, glm::vec3 axis) {
		glm::mat4 mat = glm::identity<glm::mat4>();
		mat[0][0] = -(pow(axis[1], 2) + pow(axis[2], 2)) * sin(angle);
		mat[1][0] = axis[0] * axis[1] * sin(angle) + axis[2] * cos(angle);
		mat[2][0] = axis[0] * axis[2] * sin(angle) - axis[1] * cos(angle);
		mat[0][1] = axis[0] * axis[1] * sin(angle) - axis[1] * cos(angle);
		mat[1][1] = -(pow(axis[0], 2) + pow(axis[2], 2)) * sin(angle);
		mat[2][1] = axis[1] * axis[2] * sin(angle) + axis[0] * cos(angle);
		mat[0][2] = axis[0] * axis[2] * sin(angle) + axis[1] * cos(angle);
		mat[1][2] = axis[1] * axis[2] * sin(angle) - axis[0] * cos(angle);
		mat[2][2] = -(pow(axis[0], 2) + pow(axis[1], 2)) * sin(angle);
		return mat;
	}

	void setDescentTransformArrays(IKData* dataPtr) {
		dataPtr->forwardModelSpaceTransforms = dataPtr->rigConfig->getModelSpaceTransforms(dataPtr->rigConfig->getCurrentFrameIndex(), true);
		dataPtr->jointSpaceTransforms = dataPtr->rigConfig->getJointSpaceTransforms(dataPtr->rigConfig->getCurrentFrameIndex(), true);
		IKChain ikChain;
		std::vector<glm::mat4>* bt;
		for (int c = 0; c < dataPtr->ikChains.size(); c++) {
			auto joints = dataPtr->ikChains[c]->getJoints();
			bt = &dataPtr->backwardModelSpaceTransformsPerChain[c];
			(*bt) = dataPtr->jointSpaceTransforms;
			for (int i = joints.size() - 2; 0 <= i; i--) {
				(*bt)[joints[i]] = (*bt)[joints[i]] * (*bt)[joints[i + 1]];
			}
		}
	}

	// terminos para el descenso de gradiente
	
	// termino 1 (acercar la animacion creada a la animacion original)
	std::function<float(const std::vector<float>&, IKData*)> term1Function =
		[](const std::vector<float>& varAngles, IKData* dataPtr)->float {
		float result = 0;
		for (int i = 0; i < varAngles.size(); i++) {
			result += pow(varAngles[i] - dataPtr->baseAngles[i], 2);
		}
		return dataPtr->alphaValue * result;
	};

	std::function<float(const std::vector<float>&, int, IKData*)> term1PartialDerivativeFunction =
		[](const std::vector<float>& varAngles, int varIndex, IKData* dataPtr)->float {
		return dataPtr->alphaValue * 2 * (varAngles[varIndex] - dataPtr->baseAngles[varIndex]);
	};

	// termino 2 (seguir la curva deseada para el end effector)
	std::function<float(const std::vector<float>&, IKData*)> term2Function =
		[](const std::vector<float>& varAngles, IKData* dataPtr)->float {
		float result = 0;
		int eeIndex;
		glm::vec4 baseVec(0, 0, 0, 1);
		glm::vec3 eePos;
		for (int c = 0; c < dataPtr->ikChains.size(); c++) {
			eeIndex = dataPtr->ikChains[c]->getJoints().back();
			eePos = glm::vec3(dataPtr->forwardModelSpaceTransforms[eeIndex] * baseVec);
			result += glm::length2(eePos - dataPtr->ikChains[c]->getCurrentEETarget());
		}
		return  dataPtr->betaValue * result;
	};

	std::function<float(const std::vector<float>&, int, IKData*)> term2PartialDerivativeFunction =
		[](const std::vector<float>& varAngles, int varIndex, IKData* dataPtr)->float {
		float result = 0;
		glm::mat4 TA; glm::mat4 TB; glm::vec3 TvarScl; glm::fquat TvarQuat;	glm::vec3 TvarTr;
		glm::vec3 skew;	glm::vec4 perspective;
		for (int c = 0; c < dataPtr->ikChains.size(); c++) {
			// chequeamos si el angulo variable es parte de la cadena actual
			auto joints = dataPtr->ikChains[c]->getJoints();
			int ind = funcUtils::findIndex(joints, dataPtr->jointIndexes[varIndex]);
			if (ind != -1) {
				JointIndex varJointIndex = joints[ind];
				JointIndex firstJointIndex = joints[0];
				JointIndex eeIndex = joints.back();
				glm::mat4 TvarRaw = dataPtr->jointSpaceTransforms[dataPtr->jointIndexes[varIndex]];
				glm::decompose(TvarRaw, TvarScl, TvarQuat, TvarTr, skew, perspective);
				TA = (varJointIndex != firstJointIndex ? dataPtr->forwardModelSpaceTransforms[joints[ind - 1]] :
					glm::identity<glm::mat4>()) * glmUtils::translationToMat4(TvarTr);
				TB = glmUtils::scaleToMat4(TvarScl) * (varJointIndex != eeIndex ?
					dataPtr->backwardModelSpaceTransformsPerChain[c][joints[ind + 1]] : glm::identity<glm::mat4>());
				glm::vec3 b = glm::vec3(TB * glm::vec4(0, 0, 0, 1));
				glm::mat4 Tvar = glmUtils::rotationToMat4(TvarQuat);
				glm::mat4 dTvar = rotationMatrixDerivative_dAngle(varAngles[varIndex], dataPtr->rotationAxes[dataPtr->jointIndexes[varIndex]]);
				glm::vec3 eeT = dataPtr->ikChains[c]->getCurrentEETarget();
				for (int k = 0; k <= 2; k++) {
					float mult1 = 0;
					for (int j = 0; j <= 3; j++) {
						for (int i = 0; i <= 3; i++) {
							mult1 += b[j] * TA[k][i] * Tvar[i][j] - eeT[k] / 16;
						}
					}
					float mult2 = 0;
					for (int j = 0; j <= 3; j++) {
						for (int i = 0; i <= 3; i++) {
							mult2 += b[j] * TA[k][i] * dTvar[i][j];
						}
					}
					result += mult1 * mult2;
				}
			}
		}
		return dataPtr->betaValue * 2 * result;
	};

	// termino 3 (acercar los valores actuales a los del frame anterior)
	std::function<float(const std::vector<float>&, IKData*)> term3Function =
		[](const std::vector<float>& varAngles, IKData* dataPtr)->float {
		float result = 0;
		for (int i = 0; i < varAngles.size(); i++) {
			result += pow(varAngles[i] - dataPtr->previousAngles[i], 2);
		}
		return dataPtr->gammaValue * result;
	};

	std::function<float(const std::vector<float>&, int, IKData*)> term3PartialDerivativeFunction =
		[](const std::vector<float>& varAngles, int varIndex, IKData* dataPtr)->float {
		return dataPtr->gammaValue * 2 * (varAngles[varIndex] - dataPtr->previousAngles[varIndex]);
	};

	std::function<void(std::vector<float>&, IKData*, std::vector<float>&)>  postDescentStepCustomBehaviour =
		[](std::vector<float>& args, IKData* dataPtr, std::vector<float>& argsRawDelta)->void {
		// aplicar restricciones de movimiento
		for (int i = 0; i < args.size(); i++) {
			int jIndex = dataPtr->jointIndexes[i];
			if (args[i] <= dataPtr->motionRanges[jIndex][0]) { 
				args[i] = dataPtr->motionRanges[jIndex][0];
				argsRawDelta[i] = 0;
			}
			else if (dataPtr->motionRanges[jIndex][1] < args[i]) { args[i] = dataPtr->motionRanges[jIndex][1]; }
		}
		// setear nuevos angulos
		std::vector<JointRotation>* configRot = dataPtr->rigConfig->getDynamicJointRotations(dataPtr->rigConfig->getNextFrameIndex());
		for (int i = 0; i < dataPtr->jointIndexes.size(); i++) {
			int jIndex = dataPtr->jointIndexes[i];
			(*configRot)[jIndex].setRotationAngle(args[i]);
		}
		// calcular arreglos de transformaciones
		setDescentTransformArrays(dataPtr);
	};

	InverseKinematics::InverseKinematics(IKRig* ikRig, std::vector<ChainIndex> ikChains) {
		m_ikRig = ikRig;
	}

	void InverseKinematics::init() {
		FunctionTerm<IKData> term1(term1Function, term1PartialDerivativeFunction);
		FunctionTerm<IKData> term2(term2Function, term2PartialDerivativeFunction);
		FunctionTerm<IKData> term3(term3Function, term3PartialDerivativeFunction);
		auto terms = std::vector<FunctionTerm<IKData>>({ term1, term2, term3 });
		m_gradientDescent = GradientDescent<IKData>(terms, 0, &m_ikData, postDescentStepCustomBehaviour);
		m_ikData.descentRate = 0.01;
		m_ikData.maxIterations = 100;
		m_ikData.alphaValue = 0.2f;
		m_ikData.betaValue = 0.6f;
		m_ikData.gammaValue = 0.2f;
		m_ikData.targetAngleDelta = 0.0f;
		setIKChains(m_ikChains);
	}

	void InverseKinematics::setIKChains(std::vector<ChainIndex> ikChains) {
		m_ikChains = ikChains;
		std::vector<IKChain*> chainPtrs(ikChains.size());
		for (int i = 0; i < ikChains.size(); i++) {
			chainPtrs[i] = m_ikRig->getIKChain(ikChains[i]);
		}
		m_ikData.ikChains = chainPtrs;
		std::vector<std::string> ikChainNames(chainPtrs.size());
		m_ikData.jointIndexes = {};
		std::vector<JointIndex> jointIndexes;
		for (int c = 0; c < chainPtrs.size(); c++) {
			jointIndexes.insert(jointIndexes.end(), chainPtrs[c]->getJoints().begin(), chainPtrs[c]->getJoints().end());
			ikChainNames[c] = chainPtrs[c]->getName();
			for (int i = 0; i < c; i++) {
				if (ikChainNames[i] == ikChainNames[c]) {
					MONA_LOG_ERROR("InverseKinematics: chain names must all be different.");
					return;
				}
			}
		}
		funcUtils::sortUnique(jointIndexes);
		m_ikData.jointIndexes = jointIndexes;
		m_ikData.motionRanges = std::vector<glm::vec2>(m_ikData.jointIndexes.size());
		for (int j = 0; j < m_ikData.jointIndexes.size(); j++) {
			JointIndex jInd = m_ikData.jointIndexes[j];
			m_ikData.motionRanges[j] = m_ikRig->getIKNode(jInd)->getMotionRange();
		}
		m_gradientDescent.setArgNum(m_ikData.jointIndexes.size());
		m_ikData.rotationAxes.resize(m_ikData.jointIndexes.size());
		m_ikData.baseAngles.resize(m_ikData.jointIndexes.size());
	}

	std::vector<std::pair<JointIndex, glm::fquat>> InverseKinematics::solveIKChains(AnimationIndex animationIndex) {

		m_ikData.rigConfig = m_ikRig->getAnimationConfig(animationIndex);

		std::vector<JointRotation>const& baseRotations = m_ikData.rigConfig->getBaseJointRotations(m_ikData.rigConfig->getNextFrameIndex());
		std::vector<JointRotation>* dynamicRotations = m_ikData.rigConfig->getDynamicJointRotations(m_ikData.rigConfig->getCurrentFrameIndex());
		// recuperamos los angulos previamente usados de la animacion
		m_ikData.previousAngles.resize(m_ikData.jointIndexes.size());
		for (int i = 0; i < m_ikData.jointIndexes.size(); i++) {
			m_ikData.previousAngles[i] = (*dynamicRotations)[m_ikData.jointIndexes[i]].getRotationAngle();
		}

		for (int i = 0; i < m_ikData.jointIndexes.size(); i++) {
			m_ikData.rotationAxes[i] = baseRotations[m_ikData.jointIndexes[i]].getRotationAxis();
			m_ikData.baseAngles[i] = baseRotations[m_ikData.jointIndexes[i]].getRotationAngle();
		}
		// calcular arreglos de transformaciones
		setDescentTransformArrays(&m_ikData);
		std::vector<float> computedAngles = m_gradientDescent.computeArgsMin(m_ikData.descentRate, 
			m_ikData.maxIterations, m_ikData.targetAngleDelta, m_ikData.previousAngles);
		std::vector<std::pair<JointIndex, glm::fquat>> result(computedAngles.size());
		std::vector<JointRotation>* dmicRot = m_ikData.rigConfig->getDynamicJointRotations(m_ikData.rigConfig->getNextFrameIndex());
		for (int i = 0; i < m_ikData.jointIndexes.size(); i++) {
			JointIndex jIndex = m_ikData.jointIndexes[i];
			(*dmicRot)[i].setRotationAngle(computedAngles[i]);
			result[i] = { jIndex,(* dmicRot)[i].getQuatRotation() };
		}
		return result;		
	}

	std::vector<glm::mat4> ForwardKinematics::CustomSpaceTransforms(glm::mat4 baseTransform, AnimationIndex animIndex, FrameIndex frame, bool useDynamicRotations) {
		IKRigConfig* config = m_ikRig->getAnimationConfig(animIndex);
		std::vector<glm::mat4> customSpaceTr(m_ikRig->getTopology().size());
		for (int i = 0; i < m_ikRig->getTopology().size(); i++) { customSpaceTr[i] = glm::identity<glm::mat4>(); }
		std::vector<JointRotation>const& rotations = useDynamicRotations ? (*config->getDynamicJointRotations(frame)) : config->getBaseJointRotations(frame);
		for (int i = 0; i < config->getJointIndices().size(); i++) {
			JointIndex jIndex = config->getJointIndices()[i];
			glm::mat4 baseTransform_ = i == 0 ? baseTransform : customSpaceTr[m_ikRig->getTopology()[jIndex]];
			customSpaceTr[jIndex] = baseTransform_ *
				glmUtils::translationToMat4(config->getJointPositions()[i]) *
				glmUtils::rotationToMat4(rotations[i].getQuatRotation()) *
				glmUtils::scaleToMat4(config->getJointScales()[i]);
		}
		return customSpaceTr;
	}

	glm::mat4 ForwardKinematics::CustomSpaceTransform(glm::mat4 baseTransform, JointIndex jointIndex, AnimationIndex animIndex, FrameIndex frame, bool useDynamicRotations) {
		auto config = m_ikRig->getAnimationConfig(animIndex);
		MONA_ASSERT(config->hasJoint(jointIndex), "ForwardKinematics: Joint with index {0} not present in animation.", jointIndex);
		std::vector<JointRotation>const& rotations = useDynamicRotations ? (*config->getDynamicJointRotations(frame)) : config->getBaseJointRotations(frame);
		// root
		glm::mat4 customSpaceTr = glm::identity<glm::mat4>();
		JointIndex parent = jointIndex;
		while (parent != -1) {
			customSpaceTr = glmUtils::translationToMat4(config->getJointPositions()[m_ikRig->getTopology()[parent]]) *
				glmUtils::rotationToMat4(rotations[m_ikRig->getTopology()[parent]].getQuatRotation()) *
				glmUtils::scaleToMat4(config->getJointScales()[m_ikRig->getTopology()[parent]]) *
				customSpaceTr;
				parent = m_ikRig->getTopology()[jointIndex];
		}
		return baseTransform*customSpaceTr;
	}

	std::vector<glm::vec3> ForwardKinematics::CustomSpacePositions(glm::mat4 baseTransform, AnimationIndex animIndex, FrameIndex frame, bool useDynamicRotations) {
		std::vector<glm::mat4> customSpaceTr = CustomSpaceTransforms(baseTransform, animIndex, frame, useDynamicRotations);
		std::vector<glm::vec3> customSpacePos(customSpaceTr.size());
		for (int i = 0; i < customSpaceTr.size(); i++) {
			customSpacePos[i] = customSpaceTr[i] * glm::vec4(0, 0, 0, 1);
		}
		return customSpacePos;
	}

	std::vector<glm::mat4> ForwardKinematics::ModelSpaceTransforms(AnimationIndex animIndex, FrameIndex frame, bool useDynamicRotations) {
		return CustomSpaceTransforms(glm::identity<glm::mat4>(), animIndex, frame, useDynamicRotations);
	}

	std::vector<glm::vec3> ForwardKinematics::ModelSpacePositions(AnimationIndex animIndex, FrameIndex frame, bool useDynamicRotations) {
		return CustomSpacePositions(glm::identity<glm::mat4>(), animIndex, frame, useDynamicRotations);
	}

	glm::mat4 ForwardKinematics::ModelSpaceTransform(AnimationIndex animIndex, JointIndex jointIndex, FrameIndex frame, bool useDynamicRotations) {
		return CustomSpaceTransform(glm::identity<glm::mat4>(), animIndex, jointIndex, frame, useDynamicRotations);
	}

	std::vector<glm::mat4> ForwardKinematics::JointSpaceTransforms(AnimationIndex animIndex, FrameIndex frame, bool useDynamicRotations) {
		IKRigConfig* config = m_ikRig->getAnimationConfig(animIndex);
		std::vector<glm::mat4> jointSpaceTr(m_ikRig->getTopology().size());
		for (int i = 0; i < m_ikRig->getTopology().size(); i++) { jointSpaceTr[i] = glm::identity<glm::mat4>(); }
		std::vector<JointRotation>const& rotations = useDynamicRotations ? (*config->getDynamicJointRotations(frame)) : config->getBaseJointRotations(frame);
		for (int i = 0; i < config->getJointIndices().size(); i++) {
			JointIndex jIndex = config->getJointIndices()[i];
			jointSpaceTr[jIndex] = glmUtils::translationToMat4(config->getJointPositions()[i]) *
				glmUtils::rotationToMat4(rotations[i].getQuatRotation()) *
				glmUtils::scaleToMat4(config->getJointScales()[i]);
		}
		return jointSpaceTr;
	}

}