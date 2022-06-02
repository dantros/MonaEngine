#pragma once
#ifndef IK_HPP
#define IK_HPP

#include <vector>
#include <functional>
#include "IKRig.hpp"

namespace Mona {
	template <typename outT, typename varT>
	struct FunctionTerm { // representa un termino de la funcion objetivo
		// funcion que calcula el valor del termino dado un vector con los valores de las variables
		std::function<outT(std::vector<varT>)> termValue;
		// funcion que calcula el valor de la derivada del termino,
		// dado un vector con los valores de las variables y el indice de la variable
		// respecto a la cual se deriva
		std::function<outT(std::vector<varT>, int)> termDerivativeValue;
	};

	class GradientDescentIK {
		GradientDescentIK(IKRig* ikRig);
		std::vector<FunctionTerm<float, float>> m_terms;
		IKRig* m_ikRig;
	};

	class ForwardKinematics {
		ForwardKinematics(IKRig* ikRig);
		std::vector<glm::vec3> ModelSpacePositions(AnimationIndex animIndex);
		glm::vec3 ModelSpaceJointPosition(AnimationIndex animIndex, JointIndex jointIndex);
		std::vector<glm::mat4x4> ModelSpaceTransforms(AnimationIndex animIndex);
		glm::mat4x4 ModelSpaceJointTransform(AnimationIndex animIndex, JointIndex jointIndex);

		IKRig* m_ikRig;

	};
}




#endif