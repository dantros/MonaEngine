#include "Kinematics.hpp"
#include "../Core/GlmUtils.hpp"
#include "../Core/FuncUtils.hpp"
#include <glm/gtx/matrix_decompose.hpp>
#include "IKRig.hpp"


namespace Mona {

	glm::mat4 rotationMatrixDerivative_dAngle(float angle, glm::vec3 axis) {
		// column major!!
		glm::mat4 mat(0.0f);
		mat[0][0] = sin(angle) * (pow(axis[0], 2) - 1);;
		mat[0][1] = axis[0] * axis[1] * sin(angle) + axis[2] * cos(angle);
		mat[0][2] = axis[0] * axis[2] * sin(angle) - axis[1] * cos(angle);
		mat[1][0] = axis[0] * axis[1] * sin(angle) - axis[2] * cos(angle);
		mat[1][1] = sin(angle) * (pow(axis[1], 2) - 1);;
		mat[1][2] = axis[1] * axis[2] * sin(angle) + axis[0] * cos(angle);
		mat[2][0] = axis[0] * axis[2] * sin(angle) + axis[1] * cos(angle);
		mat[2][1] = axis[1] * axis[2] * sin(angle) - axis[0] * cos(angle);
		mat[2][2] = sin(angle) * (pow(axis[2], 2) - 1);;
		return mat;
	}

	// terminos para el descenso de gradiente
	
	// termino 1 (seguir la curva deseada para el end effector)
	std::function<float(const std::vector<float>&, IKData*)> term1Function =
		[](const std::vector<float>& varAngles, IKData* dataPtr)->float {
		float result = 0;
		int eeIndex;
		glm::vec4 baseVec(0, 0, 0, 1);
		glm::vec3 eePos;
		std::vector<JointIndex> endEffectors;
		for (int c = 0; c < dataPtr->ikChains.size(); c++) {
			endEffectors.push_back(dataPtr->ikChains[c]->getEndEffector());
		}
		std::vector<glm::mat4> forwardModelSpaceTransforms = dataPtr->rigConfig->getEEListModelSpaceVariableTransforms(endEffectors);
		for (int c = 0; c < dataPtr->ikChains.size(); c++) {
			eeIndex = endEffectors[c];
			eePos = glm::vec3(forwardModelSpaceTransforms[eeIndex] * baseVec);
			result += glm::length2(eePos - dataPtr->ikChains[c]->getCurrentEETarget());
		}
		return result;
	};

	std::function<float(const std::vector<float>&, int, IKData*)> term1PartialDerivativeFunction =
		[](const std::vector<float>& varAngles, int varIndex, IKData* dataPtr)->float {
		float result = 0;
		glm::mat4 TA; glm::mat4 TB; glm::vec3 TvarScl; glm::fquat TvarQuat;	glm::vec3 TvarTr;
		glm::vec3 skew;	glm::vec4 perspective;
		// buscamos las cadenas afectadas
		std::vector<IKChain*> affectedChains;
		std::vector<int> indexes;
		std::vector<JointIndex> endEffectors;
		JointIndex varJoint = dataPtr->jointIndexes[varIndex];
		for (int i = 0; i < dataPtr->ikChains.size(); i++) {
			std::vector<JointIndex>const& joints = dataPtr->ikChains[i]->getJoints();
			int ind = funcUtils::findIndex(joints, varJoint);
			if (ind != -1) {
				affectedChains.push_back(dataPtr->ikChains[i]);
				indexes.push_back(ind);
				endEffectors.push_back(affectedChains.back()->getEndEffector());
			}
		}
		// calcular arreglos de transformaciones
		std::vector<glm::mat4> jointSpaceTransforms;
		// multiplicacion en cadena desde la raiz hasta el joint i
		std::vector<glm::mat4> forwardModelSpaceTransforms = dataPtr->rigConfig->getEEListModelSpaceVariableTransforms(endEffectors, &jointSpaceTransforms);

		// multiplicacion en cadena desde el ee de la cadena hasta el joint i
		std::vector<std::vector<glm::mat4>> backwardModelSpaceTransformsPerChain(affectedChains.size());
		std::vector<glm::mat4>* bt;
		for (int i = 0; i < affectedChains.size(); i++) {
			std::vector<JointIndex>const& joints = affectedChains[i]->getJoints();
			int ind = indexes[i];
			bt = &backwardModelSpaceTransformsPerChain[i];
			(*bt) = jointSpaceTransforms;
			for (int j = joints.size() - 2; ind + 1 <= j; j--) {
				(*bt)[joints[j]] = (*bt)[joints[j]] * (*bt)[joints[j + 1]];
			}
		}

		for (int c = 0; c < affectedChains.size(); c++) {
			std::vector<JointIndex>const& joints = affectedChains[c]->getJoints();
			int ind = indexes[c];
			// matriz de trnasformacion de la joint actual
			glm::mat4 TvarRaw = jointSpaceTransforms[varJoint];
			glm::decompose(TvarRaw, TvarScl, TvarQuat, TvarTr, skew, perspective);
			// matriz que va a la izquierda de la matriz de rotacion de la joint actual en el calculo de la posicion con FK
			JointIndex chainParent = affectedChains[c]->getParentJoint();
			glm::mat4 chainBaseTransform = chainParent == -1 ? glm::identity<glm::mat4>() :
				forwardModelSpaceTransforms[chainParent];
			TA = (0 < ind ? forwardModelSpaceTransforms[joints[ind - 1]] :
				chainBaseTransform) * glmUtils::translationToMat4(TvarTr);

			// matriz que va a la  derecha de la matriz de rotacion de la joint actual en el calculo de la posicion con FK
			TB = glmUtils::scaleToMat4(TvarScl) * (ind < joints.size() - 1 ?
				backwardModelSpaceTransformsPerChain[c][joints[ind + 1]] : glm::identity<glm::mat4>());
			glm::vec4 b = TB * glm::vec4(0, 0, 0, 1);
			glm::mat4 Tvar = glmUtils::rotationToMat4(TvarQuat);
			glm::mat4 dTvar = rotationMatrixDerivative_dAngle(varAngles[varIndex], dataPtr->rotationAxes[varIndex]);
			glm::vec4 eeT = glm::vec4(affectedChains[c]->getCurrentEETarget(), 1);
			glm::vec4 eePosCurr_d = forwardModelSpaceTransforms[joints.back()] * glm::vec4(0, 0, 0, 1);
			for (int k = 0; k <= 3; k++) {
				float mult1 = 0;
				for (int j = 0; j <= 3; j++) {
					for (int i = 0; i <= 3; i++) {
						mult1 += b[j] * TA[i][k] * Tvar[j][i] - eeT[k] / 16;
					}
				}
				float mult2 = 0;
				for (int j = 0; j <= 3; j++) {
					for (int i = 0; i <= 3; i++) {
						mult2 += b[j] * TA[i][k] * dTvar[j][i];
					}
				}
				result += mult1 * mult2;
			}
		}
		return 2 * result;
	};

	// termino 2 (acercar la animacion creada a la animacion original)
	std::function<float(const std::vector<float>&, IKData*)> term2Function =
		[](const std::vector<float>& varAngles, IKData* dataPtr)->float {
		float result = 0;
		for (int i = 0; i < varAngles.size(); i++) {
			result += pow(varAngles[i] - dataPtr->baseAngles[i], 2);
		}
		return result;
	};

	std::function<float(const std::vector<float>&, int, IKData*)> term2PartialDerivativeFunction =
		[](const std::vector<float>& varAngles, int varIndex, IKData* dataPtr)->float {
		return 2 * (varAngles[varIndex] - dataPtr->baseAngles[varIndex]);
	};

	

	// termino 3 (acercar los valores actuales a los del frame anterior)
	std::function<float(const std::vector<float>&, IKData*)> term3Function =
		[](const std::vector<float>& varAngles, IKData* dataPtr)->float {
		float result = 0;
		for (int i = 0; i < varAngles.size(); i++) {
			result += pow(varAngles[i] - dataPtr->previousAngles[i], 2);
		}
		return result;
	};

	std::function<float(const std::vector<float>&, int, IKData*)> term3PartialDerivativeFunction =
		[](const std::vector<float>& varAngles, int varIndex, IKData* dataPtr)->float {
		return 2 * (varAngles[varIndex] - dataPtr->previousAngles[varIndex]);
	};

	std::function<void(std::vector<float>&, IKData*, std::vector<float>&,int)>  postDescentStepCustomBehaviour =
		[](std::vector<float>& args, IKData* dataPtr, std::vector<float>& argsRawDelta, int varIndex_progressive)->void {
		// setear nuevos angulos
		std::vector<JointRotation>* configRot = dataPtr->rigConfig->getVariableJointRotations();
		int jIndex = dataPtr->jointIndexes[varIndex_progressive];
		(*configRot)[jIndex].setRotationAngle(args[varIndex_progressive]);
	};

	InverseKinematics::InverseKinematics(IKRig* ikRig) {
		m_ikRig = ikRig;
	}

	void InverseKinematics::init() {
		FunctionTerm<IKData> term1(term1Function, term1PartialDerivativeFunction);
		FunctionTerm<IKData> term2(term2Function, term2PartialDerivativeFunction);
		FunctionTerm<IKData> term3(term3Function, term3PartialDerivativeFunction);
		auto terms = std::vector<FunctionTerm<IKData>>({ term1,term2, term3 });
		m_gradientDescent = GradientDescent<IKData>(terms, 0, &m_ikData, postDescentStepCustomBehaviour);
		m_ikData.descentRate = 1.0f;
		m_ikData.maxIterations = 300;
		m_ikData.targetAngleDelta = 1 / pow(10, 3);
		m_gradientDescent.setTermWeight(0, 1.0f / (pow(10, 2) * m_ikRig->getRigHeight()));
		m_gradientDescent.setTermWeight(1, 0.015f);		
		m_gradientDescent.setTermWeight(2, 0.018f);
		setIKChains();
	}

	void InverseKinematics::setIKChains() {
		std::vector<IKChain*> chainPtrs(m_ikRig->getChainNum());
		for (int i = 0; i < m_ikRig->getChainNum(); i++) {
			chainPtrs[i] = m_ikRig->getIKChain(i);
		}
		m_ikData.ikChains = chainPtrs;
		m_ikData.jointIndexes = {};
		std::vector<JointIndex> jointIndexes;
		for (int c = 0; c < chainPtrs.size(); c++) {
			// se dejan fuera los ee, ya que cambiar sus angulos de rotacion no afecta su posicion
			jointIndexes.insert(jointIndexes.end(), chainPtrs[c]->getJoints().begin(), chainPtrs[c]->getJoints().end()-1);
		}
		funcUtils::removeDuplicates(jointIndexes);
		m_ikData.jointIndexes = jointIndexes;
		m_gradientDescent.setArgNum(m_ikData.jointIndexes.size());
		m_ikData.rotationAxes.resize(m_ikData.jointIndexes.size());
		m_ikData.baseAngles.resize(m_ikData.jointIndexes.size());
	}

	std::vector<std::pair<JointIndex, float>> InverseKinematics::solveIKChains(AnimationIndex animationIndex) {

		m_ikData.rigConfig = m_ikRig->getAnimationConfig(animationIndex);
		FrameIndex nextFrame = m_ikData.rigConfig->getNextFrameIndex();
		FrameIndex currentFrame = m_ikData.rigConfig->getCurrentFrameIndex();
		float currentFrameRepTime = m_ikData.rigConfig->getReproductionTime(currentFrame);

		// recuperamos los angulos previamente usados de la animacion
		m_ikData.previousAngles.resize(m_ikData.jointIndexes.size());
		for (int i = 0; i < m_ikData.jointIndexes.size(); i++) {
			JointIndex jIndex = m_ikData.jointIndexes[i];
			m_ikData.previousAngles[i] = m_ikData.rigConfig->getSavedAngles(jIndex).evalCurve(currentFrameRepTime)[0];
		}
		std::vector<float> initialArgs(m_ikData.previousAngles.size());
		for (int i = 0; i < m_ikData.previousAngles.size(); i++) {
			initialArgs[i] = m_ikData.previousAngles[i] * 0.8f;
		}

		std::vector<JointRotation>const& baseRotations_target = m_ikData.rigConfig->getBaseJointRotations(nextFrame);
		for (int i = 0; i < m_ikData.jointIndexes.size(); i++) {
			m_ikData.rotationAxes[i] = baseRotations_target[m_ikData.jointIndexes[i]].getRotationAxis();
			m_ikData.baseAngles[i] = baseRotations_target[m_ikData.jointIndexes[i]].getRotationAngle();
		}
		// setear rotaciones variables a los valores base de frame objetivo
		m_ikData.rigConfig->setVariableJointRotations(nextFrame);
		// ajustamos las rotaciones varaibles a los argumentos iniciales
		std::vector<JointRotation>* variableRotations = m_ikData.rigConfig->getVariableJointRotations();
		for (int i = 0; i < m_ikData.jointIndexes.size(); i++) {
			JointIndex jIndex = m_ikData.jointIndexes[i];
			(*variableRotations)[jIndex].setRotationAngle(initialArgs[i]);
		}
		std::vector<float> computedAngles = m_gradientDescent.computeArgsMin_progressive(m_ikData.descentRate, 
			m_ikData.maxIterations, m_ikData.targetAngleDelta, initialArgs);
		std::vector<std::pair<JointIndex, float>> result(computedAngles.size());
		
		for (int i = 0; i < m_ikData.jointIndexes.size(); i++) {
			JointIndex jIndex = m_ikData.jointIndexes[i];
			(*variableRotations)[jIndex].setRotationAngle(computedAngles[i]);
			result[i] = { jIndex, computedAngles[i]};
		}
		return result;		
	}

	ForwardKinematics::ForwardKinematics(IKRig* ikRig) {
		m_ikRig = ikRig;
	}

	std::vector<glm::mat4> ForwardKinematics::EEListCustomSpaceTransforms(std::vector<JointIndex> eeList, glm::mat4 baseTransform, AnimationIndex animIndex,
		float reproductionTime, std::vector<glm::mat4>* outEEListJointSpaceTransforms) {
		IKRigConfig* config = m_ikRig->getAnimationConfig(animIndex);
		std::vector<glm::mat4> eeListCustomSpaceTr(m_ikRig->getTopology().size(), glm::identity<glm::mat4>());
		if (outEEListJointSpaceTransforms != nullptr) {
			(*outEEListJointSpaceTransforms) = std::vector<glm::mat4>(m_ikRig->getTopology().size(), glm::identity<glm::mat4>());
		}
		funcUtils::sortUnique(eeList);
		std::vector<JointIndex> calcJoints;
		while (0 < eeList.size()) {
			JointIndex currJoint = eeList.back();
			eeList.pop_back();
			if (funcUtils::findIndex(calcJoints, currJoint) == -1) {
				std::vector<JointIndex> currJointHierarchy;
				glm::mat4 customSpaceTr = glm::identity<glm::mat4>();
				// recolectar jointSpaceTransforms
				while (currJoint != -1) {
					eeListCustomSpaceTr[currJoint] = JointSpaceTransform(animIndex, currJoint, reproductionTime);
					if (outEEListJointSpaceTransforms != nullptr) {
						(*outEEListJointSpaceTransforms)[currJoint] = eeListCustomSpaceTr[currJoint];
					}
					currJointHierarchy.insert(currJointHierarchy.begin(), currJoint);
					currJoint = m_ikRig->getTopology()[currJoint];
				}
				// calcular customSpaceTransforms
				for (int i = 0; i < currJointHierarchy.size(); i++) {
					glm::mat4 _baseTransform = i == 0 ? baseTransform : eeListCustomSpaceTr[currJointHierarchy[i - 1]];
					eeListCustomSpaceTr[currJointHierarchy[i]] = _baseTransform * eeListCustomSpaceTr[currJointHierarchy[i]];
					calcJoints.push_back(currJointHierarchy[i]);
				}
			}
		}
		return eeListCustomSpaceTr;

	}
	std::vector<glm::mat4> ForwardKinematics::EEListCustomSpaceVariableTransforms(std::vector<JointIndex> eeList, glm::mat4 baseTransform, AnimationIndex animIndex,
		std::vector<glm::mat4>* outEEListJointSpaceTransforms) {
		IKRigConfig* config = m_ikRig->getAnimationConfig(animIndex);
		std::vector<glm::mat4> eeListCustomSpaceTr(m_ikRig->getTopology().size(), glm::identity<glm::mat4>());
		if (outEEListJointSpaceTransforms != nullptr) {
			(*outEEListJointSpaceTransforms) = std::vector<glm::mat4>(m_ikRig->getTopology().size(), glm::identity<glm::mat4>());
		}
		funcUtils::sortUnique(eeList);
		std::vector<JointIndex> calcJoints;
		while (0 < eeList.size()) {
			JointIndex currJoint = eeList.back();
			eeList.pop_back();
			if (funcUtils::findIndex(calcJoints, currJoint) == -1) {
				std::vector<JointIndex> currJointHierarchy;
				glm::mat4 customSpaceTr = glm::identity<glm::mat4>();
				// recolectar jointSpaceTransforms
				while (currJoint != -1) {
					eeListCustomSpaceTr[currJoint] = JointSpaceVariableTransform(animIndex, currJoint);
					if (outEEListJointSpaceTransforms != nullptr) {
						(*outEEListJointSpaceTransforms)[currJoint] = eeListCustomSpaceTr[currJoint];
					}
					currJointHierarchy.insert(currJointHierarchy.begin(), currJoint);
					currJoint = m_ikRig->getTopology()[currJoint];
				}
				// calcular customSpaceTransforms
				for (int i = 0; i < currJointHierarchy.size(); i++) {
					glm::mat4 _baseTransform = i == 0 ? baseTransform : eeListCustomSpaceTr[currJointHierarchy[i - 1]];
					eeListCustomSpaceTr[currJointHierarchy[i]] = _baseTransform * eeListCustomSpaceTr[currJointHierarchy[i]];
					calcJoints.push_back(currJointHierarchy[i]);
				}
			}
		}
		return eeListCustomSpaceTr;

	}

	glm::mat4 ForwardKinematics::JointSpaceTransform(AnimationIndex animIndex, JointIndex jointIndex, float reproductionTime) {
		IKRigConfig* config = m_ikRig->getAnimationConfig(animIndex);
		float animTime = config->getAnimationTime(reproductionTime);
		float rotAngle = config->getSavedAngles(jointIndex).evalCurve(reproductionTime)[0];
		glm::vec3 rotAxis = config->getBaseJointRotations(config->getFrame(animTime))[jointIndex].getRotationAxis();
		glm::mat4 jointSpaceTr = glmUtils::translationToMat4(config->getJointPositions()[jointIndex]) *
			glmUtils::rotationToMat4(glm::angleAxis(rotAngle, rotAxis)) *
			glmUtils::scaleToMat4(config->getJointScales()[jointIndex]);
		return jointSpaceTr;

	}
	glm::mat4 ForwardKinematics::JointSpaceVariableTransform(AnimationIndex animIndex, JointIndex jointIndex) {
		IKRigConfig* config = m_ikRig->getAnimationConfig(animIndex);
		std::vector<JointRotation>* variableJointRotations = config->getVariableJointRotations();
		glm::mat4 jointSpaceTr = glmUtils::translationToMat4(config->getJointPositions()[jointIndex]) *
			glmUtils::rotationToMat4((*variableJointRotations)[jointIndex].getQuatRotation()) *
			glmUtils::scaleToMat4(config->getJointScales()[jointIndex]);
		return jointSpaceTr;

	}

}