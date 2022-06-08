#pragma once
#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <vector>
#include <functional>
#include "IKRig.hpp"
#include "GradientDescent.hpp"
#include "Splines.hpp"

namespace Mona {
	class ForwardKinematics {
		IKRig* m_ikRig;
	public:
		ForwardKinematics(IKRig* ikRig);
		ForwardKinematics() = default;
		std::vector<glm::vec3> ModelSpacePositions(AnimationIndex animIndex, bool useDynamicRotations);
		glm::vec3 ModelSpacePosition(AnimationIndex animIndex, JointIndex jointIndex, bool useDynamicRotations);
		std::vector<glm::mat4> ModelSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations);
		std::vector<glm::mat4> JointSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations);

	};

	struct DescentData {
		// constants data
		VectorX baseAngles;
		glm::vec3 targetEEPosition;
		float betaValue;
		// variables data
		std::vector<JointIndex> jointIndexes;
		std::vector<glm::mat4> forwardModelSpaceTransforms; // multiplicacion en cadena desde la raiz hasta el joint i
		std::vector<glm::mat4> backwardModelSpaceTransforms; // multiplicacion en cadena desde el ee hasta el joint i
		std::vector<glm::mat4> jointSpaceTransforms;
		std::vector<glm::vec3> rotationAxes;
		// other data
		std::vector<glm::vec2> motionRanges;
	};

	class InverseKinematics {
		IKRig* m_ikRig;
		ForwardKinematics m_forwardKinematics;
		GradientDescent<DescentData> m_gradientDescent;
		DescentData m_descentData;
		InverseKinematics() = default;
		InverseKinematics(IKRig* ikRig);
	};

	

	
};




#endif