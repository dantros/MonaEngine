#include "Kinematics.hpp"
#include "../Core/GlmUtils.hpp"
#include "../Core/FuncUtils.hpp"
#include <glm/gtx/matrix_decompose.hpp>
#include "IKRig.hpp"


namespace Mona {

	glm::mat4 rotationMatrixDerivative_dAngle(float angle, glm::vec3 axis) {
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
		std::vector<glm::mat4> forwardModelSpaceTransforms = dataPtr->ikAnimation->getEEListModelSpaceVariableTransforms(endEffectors);
		for (int c = 0; c < dataPtr->ikChains.size(); c++) {
			eeIndex = endEffectors[c];
			eePos = glm::vec3(forwardModelSpaceTransforms[eeIndex] * baseVec);
			result += glm::length2(eePos - dataPtr->ikChains[c]->getCurrentEETarget(dataPtr->ikAnimation->getAnimationIndex()));
		}
		return result;
	};

	std::function<float(const std::vector<float>&, int, IKData*)> term1PartialDerivativeFunction =
		[](const std::vector<float>& varAngles, int varIndex, IKData* dataPtr)->float {
		float result = 0;
		glm::mat4 TA; glm::mat4 TB; glm::vec3 TvarScl; glm::fquat TvarQuat;	glm::vec3 TvarTr;
		glm::vec3 skew;	glm::vec4 perspective;
		JointIndex varJoint = dataPtr->jointIndexes[varIndex];

		for (int c = 0; c < dataPtr->ikChains.size(); c++) {
			// chequeamos si la articulacion pertenece a la cadena actual
			IKChain* chain = dataPtr->ikChains[c];
			int ind = funcUtils::findIndex(chain->getJoints(), varJoint);
			if (ind != -1) {
				// matriz de trnasformacion de la joint actual
				glm::mat4 TvarRaw = dataPtr->jointSpaceTransforms[varJoint];
				glm::decompose(TvarRaw, TvarScl, TvarQuat, TvarTr, skew, perspective);
				JointIndex chainParent = chain->getParentJoint();
				glm::mat4 chainBaseTransform = chainParent == -1 ? glm::identity<glm::mat4>() : dataPtr->forwardModelSpaceTransforms[chainParent];
				// matriz que va a la izquierda de la matriz de rotacion de la joint actual en el calculo de la posicion con FK
				TA = (0 < ind ? dataPtr->forwardModelSpaceTransforms[chain->getJoints()[ind - 1]] :
					chainBaseTransform) * glmUtils::translationToMat4(TvarTr);

				// matriz que va a la  derecha de la matriz de rotacion de la joint actual en el calculo de la posicion con FK
				TB = glmUtils::scaleToMat4(TvarScl) * (ind < chain->getJoints().size() - 1 ?
					dataPtr->backwardModelSpaceTransformsPerChain[c][chain->getJoints()[ind + 1]] : glm::identity<glm::mat4>());
				glm::vec4 b = TB * glm::vec4(0, 0, 0, 1);
				glm::mat4 Tvar = glmUtils::rotationToMat4(TvarQuat);
				glm::mat4 dTvar = rotationMatrixDerivative_dAngle(varAngles[varIndex], dataPtr->rotationAxes[varIndex]);
				glm::vec4 eeT = glm::vec4(chain->getCurrentEETarget(dataPtr->ikAnimation->getAnimationIndex()), 1);
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

	void setDescentTransformArrays(IKData* dataPtr) {
		std::vector<JointIndex> endEffectors;
		for (int i = 0; i < dataPtr->ikChains.size(); i++) {
			endEffectors.push_back(dataPtr->ikChains[i]->getEndEffector());
		}
		// multiplicacion en cadena desde la raiz hasta el joint i
		dataPtr->forwardModelSpaceTransforms = dataPtr->ikAnimation->getEEListModelSpaceVariableTransforms(endEffectors, &(dataPtr->jointSpaceTransforms));
		// multiplicacion en cadena desde el ee de la cadena hasta el joint i
		dataPtr->backwardModelSpaceTransformsPerChain = std::vector<std::vector<glm::mat4>>(dataPtr->ikChains.size());
		std::vector<glm::mat4>* bt;
		for (int i = 0; i < dataPtr->ikChains.size(); i++) {
			std::vector<JointIndex>const& joints = dataPtr->ikChains[i]->getJoints();
			bt = &dataPtr->backwardModelSpaceTransformsPerChain[i];
			(*bt) = dataPtr->jointSpaceTransforms;
			for (int j = joints.size() - 2; 0 <= j; j--) {
				(*bt)[joints[j]] = (*bt)[joints[j]] * (*bt)[joints[j + 1]];
			}
		}

	}

	std::function<void(std::vector<float>&, IKData*, std::vector<float>&)>  postDescentStepCustomBehaviour =
		[](std::vector<float>& args, IKData* dataPtr, std::vector<float>& argsRawDelta)->void {
		// setear nuevos angulos
		std::vector<JointRotation>* varRots = dataPtr->ikAnimation->getVariableJointRotations();
		for (int i = 0; i < args.size(); i++) {
			int jIndex = dataPtr->jointIndexes[i];
			(*varRots)[jIndex].setRotationAngle(args[i]);
		}
		// setear arreglos de transformaciones
		setDescentTransformArrays(dataPtr);
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
		m_ikData.descentRate = 0.01f;
		m_ikData.maxIterations = 300;
		m_ikData.targetAngleDelta = 1 / pow(10, 3);
		float avgDeltaDist = m_ikRig->getRigHeight() / 200;
		m_gradientDescent.setTermWeight(0, 1 / (avgDeltaDist*m_ikRig->getRigHeight()));
		m_gradientDescent.setTermWeight(1, 2);		
		m_gradientDescent.setTermWeight(2, 4);
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

		m_ikData.ikAnimation = m_ikRig->getIKAnimation(animationIndex);
		FrameIndex nextFrame = m_ikData.ikAnimation->getNextFrameIndex();
		FrameIndex currentFrame = m_ikData.ikAnimation->getCurrentFrameIndex();
		float currentFrameRepTime = m_ikData.ikAnimation->getReproductionTime(currentFrame);

		// recuperamos los angulos previamente usados de la animacion
		m_ikData.previousAngles.resize(m_ikData.jointIndexes.size());
		for (int i = 0; i < m_ikData.jointIndexes.size(); i++) {
			JointIndex jIndex = m_ikData.jointIndexes[i];
			m_ikData.previousAngles[i] = m_ikData.ikAnimation->getSavedAngles(jIndex).evalCurve(currentFrameRepTime)[0];
		}
		std::vector<float> initialArgs = m_ikData.previousAngles;

		std::vector<JointRotation>const& baseRotations_target = m_ikData.ikAnimation->getOriginalJointRotations(nextFrame);
		for (int i = 0; i < m_ikData.jointIndexes.size(); i++) {
			m_ikData.rotationAxes[i] = baseRotations_target[m_ikData.jointIndexes[i]].getRotationAxis();
			m_ikData.baseAngles[i] = baseRotations_target[m_ikData.jointIndexes[i]].getRotationAngle();
		}
		// setear rotaciones variables a los valores base de frame objetivo
		m_ikData.ikAnimation->setVariableJointRotations(nextFrame);
		// ajustamos las rotaciones varaibles a los argumentos iniciales
		std::vector<JointRotation>* variableRotations = m_ikData.ikAnimation->getVariableJointRotations();
		for (int i = 0; i < m_ikData.jointIndexes.size(); i++) {
			JointIndex jIndex = m_ikData.jointIndexes[i];
			(*variableRotations)[jIndex].setRotationAngle(initialArgs[i]);
		}
		// setear arreglos de transformaciones
		setDescentTransformArrays(&m_ikData);
		std::vector<float> computedAngles = m_gradientDescent.computeArgsMin(m_ikData.descentRate, 
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

	std::vector<glm::mat4> ForwardKinematics::EEListCustomSpaceTransforms(std::vector<JointIndex> eeList, glm::mat4 baseTransform, IKAnimation* ikAnim,
		float reproductionTime, std::vector<glm::mat4>* outEEListJointSpaceTransforms) {
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
					eeListCustomSpaceTr[currJoint] = JointSpaceTransform(ikAnim, currJoint, reproductionTime);
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
	std::vector<glm::mat4> ForwardKinematics::EEListCustomSpaceVariableTransforms(std::vector<JointIndex> eeList, glm::mat4 baseTransform, IKAnimation* ikAnim,
		std::vector<glm::mat4>* outEEListJointSpaceTransforms) {
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
					eeListCustomSpaceTr[currJoint] = JointSpaceVariableTransform(ikAnim, currJoint);
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

	glm::mat4 ForwardKinematics::JointSpaceTransform(IKAnimation* ikAnim, JointIndex jointIndex, float reproductionTime) {
		float animTime = ikAnim->getAnimationTime(reproductionTime);
		float rotAngle = ikAnim->getSavedAngles(jointIndex).evalCurve(reproductionTime)[0];
		glm::vec3 rotAxis = ikAnim->getOriginalJointRotations(ikAnim->getFrame(animTime))[jointIndex].getRotationAxis();
		glm::mat4 jointSpaceTr = glmUtils::translationToMat4(ikAnim->getJointPosition(jointIndex)) *
			glmUtils::rotationToMat4(glm::angleAxis(rotAngle, rotAxis)) *
			glmUtils::scaleToMat4(ikAnim->getJointScale(jointIndex));
		return jointSpaceTr;

	}
	glm::mat4 ForwardKinematics::JointSpaceVariableTransform(IKAnimation* ikAnim, JointIndex jointIndex) {
		std::vector<JointRotation>* variableJointRotations = ikAnim->getVariableJointRotations();
		glm::mat4 jointSpaceTr = glmUtils::translationToMat4(ikAnim->getJointPosition(jointIndex)) *
			glmUtils::rotationToMat4((*variableJointRotations)[jointIndex].getQuatRotation()) *
			glmUtils::scaleToMat4(ikAnim->getJointScale(jointIndex));
		return jointSpaceTr;

	}

}