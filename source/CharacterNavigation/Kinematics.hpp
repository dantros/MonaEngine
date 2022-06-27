#pragma once
#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <vector>
#include <functional>
#include "GradientDescent.hpp"
#include "glm/glm.hpp"

namespace Mona {
	typedef int AnimationIndex;
	typedef int JointIndex;

	class IKRig;
	class IKChain;
	class IKRigConfig;

	class ForwardKinematics {
		IKRig* m_ikRig;
	public:
		ForwardKinematics(IKRig* ikRig);
		ForwardKinematics() = default;
		std::vector<glm::mat4> ModelSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations);
		std::vector<glm::vec3> ModelSpacePositions(AnimationIndex animIndex, bool useDynamicRotations);
		std::vector<glm::vec3> BaseModelSpacePositions(AnimationIndex animIndex, int frame);
		std::vector<glm::mat4> JointSpaceTransforms(AnimationIndex animIndex, bool useDynamicRotations);

	};

	struct IKData {
		// constants data
		VectorX baseAngles;
		float betaValue;
		// variables data
		std::vector<JointIndex> jointIndexes;
		std::vector<glm::mat4> forwardModelSpaceTransforms; // multiplicacion en cadena desde la raiz hasta el joint i
		std::vector<std::vector<glm::mat4>> backwardModelSpaceTransformsPerChain; // multiplicacion en cadena desde el ee de la cadena hasta el joint i
		std::vector<glm::mat4> jointSpaceTransforms;
		std::vector<glm::vec3> rotationAxes;
		std::vector<IKChain*> ikChains;
		// other data
		IKRigConfig* rigConfig;
		std::vector<glm::vec2> motionRanges;
		VectorX previousAngles;
		float descentRate;
		int maxIterations;
	};

	class InverseKinematics {
		IKRig* m_ikRig;
		GradientDescent<IKData> m_gradientDescent;
		IKData m_ikData;
		std::vector<std::string> m_ikChainNames;
		std::vector<VectorX> m_previousAnglesPerAnimation;
	public:
		InverseKinematics() = default;
		InverseKinematics(IKRig* ikRig, std::vector<IKChain*> ikChains);
		void setIKChains(std::vector<IKChain*> ikChains);
		std::vector<std::pair<JointIndex, glm::fquat>> solveIKChains(AnimationIndex animationIndex);
	};

	

	
};




#endif