#include "Kinematics.hpp"
#include "../Core/GlmUtils.hpp"


namespace Mona {

	ForwardKinematics::ForwardKinematics(IKRig* ikRig) {
		m_ikRig = ikRig;
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
			return 2 * (std::abs(varAngles[varIndex] - dataPtr->baseAngles[varIndex]));
		};

		FunctionTerm<DescentData> term1(term1Function, term1PartialDerivativeFunction);

		// termino 2 (seguir la curva deseada para el end effector)
		std::function<float(const VectorX&, DescentData*)> term2Function;
		std::function<float(const VectorX&, int, DescentData*)> term2PartialDerivativeFunction;
		FunctionTerm<DescentData> term2(term2Function, term2PartialDerivativeFunction);

		auto terms = std::vector<FunctionTerm<DescentData>>({ term1, term2 });
		m_gradientDescent = GradientDescent<DescentData>(terms,1,&m_descentData);
	}

	

	std::vector<glm::vec3> ForwardKinematics::ModelSpacePositions(AnimationIndex animIndex, bool useDynamicRotations) {
		std::vector<glm::vec3> modelSpacePos(m_ikRig->getTopology().size());
		auto anim = m_ikRig->getAnimation(animIndex);
		auto& config = m_ikRig->getAnimationConfig(animIndex);
		auto& rotations = useDynamicRotations ? config.getDynamicJointRotations() : config.getBaseJointRotations();
		modelSpacePos[0] = { 0,0,0 };
		// root
		modelSpacePos[0] = modelSpacePos[0] * glmUtils::scaleToMat3(config.getJointScales()[0])*
			glmUtils::rotationToMat3(config.getBaseJointRotations()[0].getQuatRotation()) + config.getJointPositions()[0];
		for (int i = 1; i < m_ikRig->getTopology().size(); i++) {
			modelSpacePos[i] = modelSpacePos[m_ikRig->getTopology()[i]] * glmUtils::scaleToMat3(config.getJointScales()[i]) *
				glmUtils::rotationToMat3(config.getBaseJointRotations()[i].getQuatRotation()) + config.getJointPositions()[i];
		}
		return modelSpacePos;
	}

	std::vector<glm::mat4x4> ForwardKinematics::ModelSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations) {
		std::vector<glm::mat4x4> modelSpaceTr(m_ikRig->getTopology().size());
		auto anim = m_ikRig->getAnimation(animIndex);
		auto& config = m_ikRig->getAnimationConfig(animIndex);
		auto& rotations = useDynamicRotations ? config.getDynamicJointRotations() : config.getBaseJointRotations();
		// root
		modelSpaceTr[0] = glmUtils::scaleToMat4(config.getJointScales()[0])*
			glmUtils::rotationToMat4(rotations[0].getQuatRotation())*glmUtils::translationToMat4(config.getJointPositions()[0]);
		for (int i = 1; i < m_ikRig->getTopology().size(); i++) {
			modelSpaceTr[i] = modelSpaceTr[m_ikRig->getTopology()[i]]* glmUtils::scaleToMat4(config.getJointScales()[i]) *
				glmUtils::rotationToMat4(rotations[i].getQuatRotation()) * glmUtils::translationToMat4(config.getJointPositions()[i]);
		}
		return modelSpaceTr;
	}

	std::vector<glm::mat4x4> ForwardKinematics::JointSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations) {
		std::vector<glm::mat4x4> jointSpaceTr(m_ikRig->getTopology().size());
		auto anim = m_ikRig->getAnimation(animIndex);
		auto& config = m_ikRig->getAnimationConfig(animIndex);
		auto& rotations = useDynamicRotations ? config.getDynamicJointRotations() : config.getBaseJointRotations();
		for (int i = 0; i < m_ikRig->getTopology().size(); i++) {
			jointSpaceTr[i] = glmUtils::scaleToMat4(config.getJointScales()[i]) *glmUtils::rotationToMat4(rotations[i].getQuatRotation()) * 
				glmUtils::translationToMat4(config.getJointPositions()[i]);
		}
		return jointSpaceTr;
	}

}