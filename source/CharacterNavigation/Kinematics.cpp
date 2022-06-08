#include "Kinematics.hpp"
#include "../Core/GlmUtils.hpp"
#include <glm/gtx/matrix_decompose.hpp>


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


	InverseKinematics::InverseKinematics(IKRig* ikRig) {
		m_ikRig = ikRig;
		m_forwardKinematics = ForwardKinematics(ikRig);
		
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
			glm::vec4 baseVec(0,0,0,1);
			int eeIndex = dataPtr->jointIndexes.back();
			glm::vec3 eePos = glmUtils::vec4ToVec3(dataPtr->forwardModelSpaceTransforms[eeIndex]*baseVec);
			return  dataPtr->betaValue * glm::length2(eePos - dataPtr->targetEEPosition);
		};

		std::function<float(const VectorX&, int, DescentData*)> term2PartialDerivativeFunction =
			[](const VectorX& varAngles, int varIndex, DescentData* dataPtr)->float {
			glm::mat4 TA = 0<varIndex ? 
				dataPtr->forwardModelSpaceTransforms[dataPtr->jointIndexes[varIndex-1]] : glm::identity<glm::mat4>();
			glm::mat4 TB = varIndex<(dataPtr->jointIndexes.size()-1) ?
				dataPtr->backwardModelSpaceTransforms[dataPtr->jointIndexes[varIndex + 1]] : glm::identity<glm::mat4>();
			glm::mat4 TvarRaw = dataPtr->jointSpaceTransforms[dataPtr->jointIndexes[varIndex]];
			glm::vec3 TvarScl;
			glm::fquat Tvar;
			glm::vec3 TvarTr;
			glm::vec3 skew;
			glm::vec4 perspective;
			glm::decompose(TvarRaw, TvarScl, Tvar, TvarTr, skew, perspective);
			TA = TA * glmUtils::translationToMat4(TvarTr);
			TB = glmUtils::scaleToMat4(TvarScl) * TB;
			glm::vec3 b = glmUtils::vec4ToVec3(TB* glm::vec4(0,0,0,1));
			glm::mat4 dTvar = rotationMatrixDerivative_dAngle(varAngles[varIndex], dataPtr->rotationAxes[dataPtr->jointIndexes[varIndex]]);

			return 0;
		};
		FunctionTerm<DescentData> term2(term2Function, term2PartialDerivativeFunction);

		auto terms = std::vector<FunctionTerm<DescentData>>({ term1, term2 });
		m_gradientDescent = GradientDescent<DescentData>(terms,1,&m_descentData);
	}

	glm::vec3 ForwardKinematics::ModelSpacePosition(AnimationIndex animIndex, JointIndex jointIndex, bool useDynamicRotations) {
		glm::mat4 modelSpaceTr = glm::identity<glm::mat4>();
		auto& config = m_ikRig->getAnimationConfig(animIndex);
		auto& rotations = useDynamicRotations ? config.getDynamicJointRotations() : config.getBaseJointRotations();
		auto currentNode = m_ikRig->getIKNode(jointIndex);
		while (currentNode != nullptr) {
			int jIndex = currentNode->getIndex();
			modelSpaceTr = 	glmUtils::translationToMat4(config.getJointPositions()[jIndex]) * 
				glmUtils::rotationToMat4(rotations[jIndex].getQuatRotation()) *
				glmUtils::scaleToMat4(config.getJointScales()[jIndex]) *
				modelSpaceTr;
			currentNode = currentNode->getParent();
		}
		return glmUtils::vec4ToVec3(modelSpaceTr* glm::vec4(0, 0, 0, 1));
	}

	std::vector<glm::mat4> ForwardKinematics::ModelSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations) {
		std::vector<glm::mat4> modelSpaceTr(m_ikRig->getTopology().size());
		auto& config = m_ikRig->getAnimationConfig(animIndex);
		auto& rotations = useDynamicRotations ? config.getDynamicJointRotations() : config.getBaseJointRotations();
		// root
		modelSpaceTr[0] = glmUtils::translationToMat4(config.getJointPositions()[0]) *
			glmUtils::rotationToMat4(rotations[0].getQuatRotation()) *
			glmUtils::scaleToMat4(config.getJointScales()[0]);
			for (int i = 1; i < m_ikRig->getTopology().size(); i++) {
				modelSpaceTr[i] = modelSpaceTr[m_ikRig->getTopology()[i]] *
					glmUtils::translationToMat4(config.getJointPositions()[i]) *
					glmUtils::rotationToMat4(rotations[i].getQuatRotation()) *
					glmUtils::scaleToMat4(config.getJointScales()[i]);
			}
		return modelSpaceTr;
	}

	std::vector<glm::mat4> ForwardKinematics::JointSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations) {
		std::vector<glm::mat4> jointSpaceTr(m_ikRig->getTopology().size());
		auto anim = m_ikRig->getAnimation(animIndex);
		auto& config = m_ikRig->getAnimationConfig(animIndex);
		auto& rotations = useDynamicRotations ? config.getDynamicJointRotations() : config.getBaseJointRotations();
		for (int i = 0; i < m_ikRig->getTopology().size(); i++) {
			jointSpaceTr[i] = glmUtils::translationToMat4(config.getJointPositions()[i]) *
				glmUtils::rotationToMat4(rotations[i].getQuatRotation()) *
				glmUtils::scaleToMat4(config.getJointScales()[i]);
		}
		return jointSpaceTr;
	}

}