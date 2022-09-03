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
	class IKRigConfig;

	class ForwardKinematics {
		IKRig* m_ikRig;
	public:
		ForwardKinematics(IKRig* ikRig);
		ForwardKinematics() = default;
		std::vector<glm::mat4> CustomSpaceTransforms(glm::mat4 baseTransform, AnimationIndex animIndex, FrameIndex frame, bool useDynamicRotations);
		std::vector<glm::mat4> EEListCustomSpaceTransforms(std::vector<JointIndex> eeList, glm::mat4 baseTransform, AnimationIndex animIndex, 
			FrameIndex frame, bool useDynamicRotations, std::vector<glm::mat4>* outJointSpaceTransforms=nullptr);
		std::vector<glm::vec3> CustomSpacePositions(glm::mat4 baseTransform, AnimationIndex animIndex, FrameIndex frame, bool useDynamicRotations);
		std::vector<glm::mat4> ModelSpaceTransforms(AnimationIndex animIndex, FrameIndex frame, bool useDynamicRotations);
		std::vector<glm::vec3> ModelSpacePositions(AnimationIndex animIndex, FrameIndex frame, bool useDynamicRotations);
		std::vector<glm::mat4> EEListJointSpaceTransforms(std::vector<JointIndex> eeList, AnimationIndex animIndex, FrameIndex frame, bool useDynamicRotations);
		std::vector<glm::mat4> JointSpaceTransforms(AnimationIndex animIndex, FrameIndex frame, bool useDynamicRotations);
		glm::mat4 JointSpaceTransform(AnimationIndex animIndex, JointIndex jointIndex, FrameIndex frame, bool useDynamicRotations);

	};

	struct IKData {
		// constants data
		std::vector<float> baseAngles;
		// variables data
		std::vector<JointIndex> jointIndexes;
		std::vector<glm::vec3> rotationAxes;
		std::vector<glm::vec2> motionRanges;
		std::vector<IKChain*> ikChains;
		// other data
		IKRigConfig* rigConfig;
		std::vector<float> previousAngles;
		float descentRate;
		float targetAngleDelta;
		int maxIterations;
		FrameIndex targetFrame;
		std::vector<int> stepsByJoint;
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
		std::vector<std::pair<JointIndex, glm::fquat>> solveIKChains(AnimationIndex animationIndex, FrameIndex targetFrame);
	};

	

	
};




#endif