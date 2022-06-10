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

	void setDescentTransformArrays(DescentData* dataPtr) {
		dataPtr->forwardModelSpaceTransforms = dataPtr->rigConfig->getModelSpaceTransforms(true);
		dataPtr->jointSpaceTransforms = dataPtr->rigConfig->getJointSpaceTransforms(true);
		IKChain ikChain;
		std::vector<glm::mat4>* bt;
		for (int c = 0; c < dataPtr->ikChains.size(); c++) {
			bt = &dataPtr->backwardModelSpaceTransformsPerChain[c];
			(*bt) = dataPtr->jointSpaceTransforms;
			ikChain = dataPtr->ikChains[c];
			for (int i = ikChain.joints.size() - 2; 0 <= i; i--) {
				(*bt)[ikChain.joints[i]] = (*bt)[ikChain.joints[i]] * (*bt)[ikChain.joints[i + 1]];
			}
		}
	}

	InverseKinematics::InverseKinematics(IKRig* ikRig, std::vector<IKChain> ikChains, AnimationIndex animIndex) {
		m_ikRig = ikRig;
		//creamos terminos para el descenso de gradiente
		// termino 1 (acercar la animacion creada a la animacion original)
		std::function<float(const VectorX&, DescentData*)> term1Function =
			[](const VectorX& varAngles, DescentData* dataPtr)->float{
			return (varAngles - dataPtr->baseAngles).squaredNorm();
		};

		std::function<float(const VectorX&, int, DescentData*)> term1PartialDerivativeFunction =
			[](const VectorX& varAngles, int varIndex, DescentData* dataPtr)->float {
			return 2 * (varAngles[varIndex] - dataPtr->baseAngles[varIndex]);
		};

		FunctionTerm<DescentData> term1(term1Function, term1PartialDerivativeFunction);

		// termino 2 (seguir la curva deseada para el end effector)
		std::function<float(const VectorX&, DescentData*)> term2Function =
			[](const VectorX& varAngles, DescentData* dataPtr)->float {
			float result = 0;
			int eeIndex;
			glm::vec4 baseVec(0, 0, 0, 1);
			glm::vec3 eePos;
			for (int c = 0; c < dataPtr->ikChains.size(); c++) {
				eeIndex = dataPtr->ikChains[c].joints.back();
				eePos = glmUtils::vec4ToVec3(dataPtr->forwardModelSpaceTransforms[eeIndex] * baseVec);
				result += glm::length2(eePos - dataPtr->targetEEPositions[c]);
			}
			result = dataPtr->betaValue * result;
			return result;
		};

		std::function<float(const VectorX&, int, DescentData*)> term2PartialDerivativeFunction =
			[](const VectorX& varAngles, int varIndex, DescentData* dataPtr)->float {
			float result = 0;
			glm::mat4 TA;
			glm::mat4 TB;
			glm::vec3 TvarScl;
			glm::fquat TvarQuat;
			glm::vec3 TvarTr;
			glm::vec3 skew;
			glm::vec4 perspective;
			for (int c = 0; c < dataPtr->ikChains.size(); c++) {
				// chequeamos si el angulo variable es parte de la cadena actual
				int ind = funcUtils::findIndex(dataPtr->ikChains[c].joints, dataPtr->jointIndexes[varIndex]);
				if (ind != -1) {
					JointIndex varJointIndex = dataPtr->ikChains[c].joints[ind];
					JointIndex baseJointIndex = dataPtr->ikChains[c].joints[0];
					JointIndex eeIndex = dataPtr->ikChains[c].joints.back();
					glm::mat4 TvarRaw = dataPtr->jointSpaceTransforms[dataPtr->jointIndexes[varIndex]];
					glm::decompose(TvarRaw, TvarScl, TvarQuat, TvarTr, skew, perspective);
					TA = (varJointIndex != baseJointIndex ? dataPtr->forwardModelSpaceTransforms[dataPtr->ikChains[c].joints[ind-1]] : 
						glm::identity<glm::mat4>()) * glmUtils::translationToMat4(TvarTr);
					TB = glmUtils::scaleToMat4(TvarScl) * (varJointIndex != eeIndex ?	
						dataPtr->backwardModelSpaceTransformsPerChain[c][dataPtr->ikChains[c].joints[ind+1]] :	glm::identity<glm::mat4>());
					glm::vec3 b = glmUtils::vec4ToVec3(TB * glm::vec4(0, 0, 0, 1));
					glm::mat4 Tvar = glmUtils::rotationToMat4(TvarQuat);
					glm::mat4 dTvar = rotationMatrixDerivative_dAngle(varAngles[varIndex], dataPtr->rotationAxes[dataPtr->jointIndexes[varIndex]]);
					glm::vec3 eeT = dataPtr->targetEEPositions[c];
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
			result = dataPtr->betaValue * 2 * result;
			return result;
		};
		FunctionTerm<DescentData> term2(term2Function, term2PartialDerivativeFunction);
		std::function<void(VectorX&, DescentData*)>  postDescentStepCustomBehaviour = [](VectorX& args, DescentData* dataPtr)->void {
			// aplicar restricciones de movimiento
			for (int i = 0; i < args.size(); i++) {
				int jIndex = dataPtr->jointIndexes[i];
				if (args[i] < dataPtr->motionRanges[jIndex][0]) { args[i] = dataPtr->motionRanges[jIndex][0];}
				else if (dataPtr->motionRanges[jIndex][1] < args[i]) {args[i] = dataPtr->motionRanges[jIndex][1];}
			}
			// setear nuevos angulos
			auto configRot = dataPtr->rigConfig->getDynamicJointRotationsPtr();
			for (int i = 0; i < dataPtr->jointIndexes.size(); i++) {
				int jIndex = dataPtr->jointIndexes[i];
				(*configRot)[jIndex].setRotationAngle(args[i]);
			}
			// calcular arreglos de transformaciones
			setDescentTransformArrays(dataPtr);
		};

		auto terms = std::vector<FunctionTerm<DescentData>>({ term1, term2 });
		m_gradientDescent = GradientDescent<DescentData>(terms,0,&m_descentData, postDescentStepCustomBehaviour);
		setIKChains(ikChains);
		setAnimationIndex(animIndex);
	}

	void InverseKinematics::setIKChains(std::vector<IKChain> ikChains) {
		m_descentData.ikChains = ikChains;
		m_descentData.targetEEPositions = std::vector<glm::vec3>(ikChains.size());
		m_ikChainNames = std::vector<std::string>(ikChains.size());
		m_descentData.jointIndexes = {};
		std::vector<JointIndex> jointIndexes;
		for (int c = 0; c < ikChains.size(); c++) {
			jointIndexes.insert(jointIndexes.end(), ikChains[c].joints.begin(), ikChains[c].joints.end());
			m_ikChainNames[c] = ikChains[c].name;
			for (int i = 0; i < c; i++) {
				if (m_ikChainNames[i] == m_ikChainNames[c]) {
					MONA_LOG_ERROR("InverseKinematics: chain names must all be different.");
					return;
				}
			}
		}
		std::sort(jointIndexes.begin(), jointIndexes.end());
		JointIndex last = -1;
		for (int j = 0; j < jointIndexes.size(); j++) {
			if (jointIndexes[j] != last) {
				m_descentData.jointIndexes.push_back(jointIndexes[j]);
			}
			last = jointIndexes[j];
		}
		m_descentData.motionRanges = std::vector<glm::vec2>(m_descentData.jointIndexes.size());
		for (int j = 0; j < m_descentData.jointIndexes.size(); j++) {
			JointIndex jInd = m_descentData.jointIndexes[j];
			m_descentData.motionRanges[j] = m_ikRig->getIKNode(jInd)->getMotionRange();
		}
		m_gradientDescent.setArgNum(m_descentData.jointIndexes.size());
		m_descentData.previousAngles = {};
		m_descentData.rotationAxes.resize(m_descentData.jointIndexes.size());
		m_descentData.baseAngles.resize(m_descentData.jointIndexes.size());
	}

	void InverseKinematics::setAnimationIndex(AnimationIndex animationIndex) {
		m_animationIndex = animationIndex;
		m_descentData.rigConfig = m_ikRig->getAnimationConfig(animationIndex);
		m_descentData.previousAngles = {};
	}

	std::vector<std::pair<JointIndex, glm::fquat>> InverseKinematics::computeRotations(std::vector<std::pair<std::string, glm::vec3>> eeTargetPerChain) {
		for (int i = 0; i < eeTargetPerChain.size(); i++) {
			int ind = funcUtils::findIndex(m_ikChainNames, eeTargetPerChain[i].first);
			MONA_ASSERT(i == -1, "InverseKinematics: input chain names don't fit the ones that were set.");
			m_descentData.targetEEPositions[ind] = eeTargetPerChain[i].second;
		}

		auto& baseRotations = m_descentData.rigConfig->getBaseJointRotations();
		// si no tenemos info de la configuracion calculada previa, tomamos la configuracion base del tiempo actual
		if (m_descentData.previousAngles.size() == 0) {
			m_descentData.previousAngles.resize(m_descentData.jointIndexes.size());
			for (int i = 0; i < m_descentData.jointIndexes.size(); i++) {
				m_descentData.previousAngles[i] = baseRotations[m_descentData.jointIndexes[i]].getRotationAngle();
			}
		}

		for (int i = 0; i < m_descentData.jointIndexes.size(); i++) {
			m_descentData.rotationAxes[i] = baseRotations[m_descentData.jointIndexes[i]].getRotationAxis();
			m_descentData.baseAngles[i] = baseRotations[m_descentData.jointIndexes[i]].getRotationAngle();
		}
		// calcular arreglos de transformaciones
		setDescentTransformArrays(&m_descentData);
		VectorX computedAngles = m_gradientDescent.computeArgsMin(m_descentData.descentRate, m_descentData.maxIterations, m_descentData.previousAngles);
		m_descentData.previousAngles = computedAngles;
		std::vector<std::pair<JointIndex, glm::fquat>> result(computedAngles.size());
		auto dmicRot = m_descentData.rigConfig->getDynamicJointRotations();
		for (int i = 0; i < m_descentData.jointIndexes.size(); i++) {
			JointIndex jIndex = m_descentData.jointIndexes[i];
			dmicRot[i].setRotationAngle(computedAngles[i]);
			result[i] = { jIndex, dmicRot[i].getQuatRotation() };
		}
		return result;		
	}







	glm::vec3 ForwardKinematics::ModelSpacePosition(AnimationIndex animIndex, JointIndex jointIndex, bool useDynamicRotations) {
		glm::mat4 modelSpaceTr = glm::identity<glm::mat4>();
		auto config = m_ikRig->getAnimationConfig(animIndex);
		auto& rotations = useDynamicRotations ? config->getDynamicJointRotations() : config->getBaseJointRotations();
		auto currentNode = m_ikRig->getIKNode(jointIndex);
		while (currentNode != nullptr) {
			int jIndex = currentNode->getIndex();
			modelSpaceTr = 	glmUtils::translationToMat4(config->getJointPositions()[jIndex]) * 
				glmUtils::rotationToMat4(rotations[jIndex].getQuatRotation()) *
				glmUtils::scaleToMat4(config->getJointScales()[jIndex]) *
				modelSpaceTr;
			currentNode = currentNode->getParent();
		}
		return glmUtils::vec4ToVec3(modelSpaceTr* glm::vec4(0, 0, 0, 1));
	}

	std::vector<glm::mat4> ForwardKinematics::ModelSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations) {
		std::vector<glm::mat4> modelSpaceTr(m_ikRig->getTopology().size());
		auto config = m_ikRig->getAnimationConfig(animIndex);
		auto& rotations = useDynamicRotations ? config->getDynamicJointRotations() : config->getBaseJointRotations();
		// root
		modelSpaceTr[0] = glmUtils::translationToMat4(config->getJointPositions()[0]) *
			glmUtils::rotationToMat4(rotations[0].getQuatRotation()) *
			glmUtils::scaleToMat4(config->getJointScales()[0]);
			for (int i = 1; i < m_ikRig->getTopology().size(); i++) {
				modelSpaceTr[i] = modelSpaceTr[m_ikRig->getTopology()[i]] *
					glmUtils::translationToMat4(config->getJointPositions()[i]) *
					glmUtils::rotationToMat4(rotations[i].getQuatRotation()) *
					glmUtils::scaleToMat4(config->getJointScales()[i]);
			}
		return modelSpaceTr;
	}

	std::vector<glm::mat4> ForwardKinematics::JointSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations) {
		std::vector<glm::mat4> jointSpaceTr(m_ikRig->getTopology().size());
		auto anim = m_ikRig->getAnimation(animIndex);
		auto config = m_ikRig->getAnimationConfig(animIndex);
		auto& rotations = useDynamicRotations ? config->getDynamicJointRotations() : config->getBaseJointRotations();
		for (int i = 0; i < m_ikRig->getTopology().size(); i++) {
			jointSpaceTr[i] = glmUtils::translationToMat4(config->getJointPositions()[i]) *
				glmUtils::rotationToMat4(rotations[i].getQuatRotation()) *
				glmUtils::scaleToMat4(config->getJointScales()[i]);
		}
		return jointSpaceTr;
	}

}