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
		std::vector<glm::mat4> forwardModelSpaceTransforms;
		std::vector<glm::mat4> backwardModelSpaceTransforms;
		ChainData chainData;
		std::vector<JointIndex> jointIndexes;
		IKRigConfig* rigConfig;
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