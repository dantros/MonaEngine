#pragma once
#ifndef IK_HPP
#define IK_HPP

#include <vector>
#include <functional>
#include "IKRig.hpp"

namespace Mona {
	template <typename outT, typename varT, typename dataT>
	struct FunctionTerm { // representa un termino de la funcion objetivo
		// funcion que calcula el valor del termino dado un vector con los valores de las variables
		std::function<outT(std::vector<varT>, dataT* dataPtr)> termValue;
		// funcion que calcula el valor de la derivada del termino,
		// dado un vector con los valores de las variables y el indice de la variable
		// respecto a la cual se deriva
		std::function<outT(std::vector<varT>, int, dataT* dataPtr)> termDerivativeValue;
	};

	class GradientDescentIK {
		friend class Kinematics;
		GradientDescentIK() = default;
		GradientDescentIK(IKRig* ikRig);
		std::vector<FunctionTerm<float, float, IKRig>> m_terms;
	};

	class Kinematics {
		Kinematics(IKRig* ikRig);
		GradientDescentIK m_gradientDescent;
		IKRig* m_ikRig;
		std::vector<glm::vec3> ModelSpacePositions(AnimationIndex animIndex, bool useDynamicRotations);
		glm::vec3 ModelSpaceJointPosition(AnimationIndex animIndex, JointIndex jointIndex, bool useDynamicRotations);
		std::vector<glm::mat4x4> ModelSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations);
		glm::mat4x4 ModelSpaceJointTransform(AnimationIndex animIndex, JointIndex jointIndex, bool useDynamicRotations);

	};

	
};




#endif