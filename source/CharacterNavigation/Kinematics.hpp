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
		std::vector<glm::mat4x4> ModelSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations);
		std::vector<glm::mat4x4> JointSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations);

	};

	struct DescentData {
		VectorX baseAngles;
		Vector3 targetEEPosition;

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