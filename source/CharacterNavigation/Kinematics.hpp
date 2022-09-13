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
	typedef int ChainIndex;
	typedef int FrameIndex;

	class IKRig;
	class IKChain;
	class IKAnimation;

	class ForwardKinematics {
		IKRig* m_ikRig;
	public:
		ForwardKinematics(IKRig* ikRig);
		ForwardKinematics() = default;
		std::vector<glm::mat4> EEListCustomSpaceTransforms(std::vector<JointIndex> eeList, glm::mat4 baseTransform, AnimationIndex animIndex,
			float reproductionTime, std::vector<glm::mat4>* outEEListJointSpaceTransforms = nullptr);
		std::vector<glm::mat4> EEListCustomSpaceVariableTransforms(std::vector<JointIndex> eeList, glm::mat4 baseTransform, AnimationIndex animIndex,
			std::vector<glm::mat4>* outEEListJointSpaceTransforms = nullptr);
		glm::mat4 JointSpaceTransform(AnimationIndex animIndex, JointIndex jointIndex, float reproductionTime);
		glm::mat4 JointSpaceVariableTransform(AnimationIndex animIndex, JointIndex jointIndex);


	};

	struct IKData {
		// constants data
		std::vector<float> baseAngles;
		// variables data
		std::vector<JointIndex> jointIndexes;
		std::vector<glm::vec3> rotationAxes;
		std::vector<IKChain*> ikChains;
		// transformaciones locales
		std::vector<glm::mat4> jointSpaceTransforms;
		// multiplicacion en cadena desde la raiz hasta el joint i
		std::vector<glm::mat4> forwardModelSpaceTransforms;
		// multiplicacion en cadena desde el ee de la cadena hasta el joint i
		std::vector<std::vector<glm::mat4>> backwardModelSpaceTransformsPerChain;
		// other data
		IKAnimation* rigConfig;
		std::vector<float> previousAngles;
		float descentRate;
		float targetAngleDelta;
		int maxIterations;
	};

	class InverseKinematics {
		IKRig* m_ikRig;
		GradientDescent<IKData> m_gradientDescent;
		IKData m_ikData;
		void setIKChains();
	public:
		InverseKinematics() = default;
		InverseKinematics(IKRig* ikRig);
		void init();
		std::vector<std::pair<JointIndex, float>> solveIKChains(AnimationIndex animationIndex);
	};

	

	
};




#endif