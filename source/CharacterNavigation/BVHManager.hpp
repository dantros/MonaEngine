#pragma once
#ifndef BVHMANAGER_HPP
#define BVHMANAGER_HPP
#include "bvh_python/cython_interface.h"
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "../Animation/AnimationController.hpp"
#include "../Core/RootDirectory.hpp"

namespace Mona {
    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXf;
    typedef Eigen::Matrix<float, 1, Eigen::Dynamic> VectorXf;

    class BVHData {
        private:
            friend class BVHManager;
            friend class IKRig;
            BVHData(std::string modelName, std::string animName);
            BVHData(std::string modelName, std::string animName, std::vector<std::string> jointNames);
            void initFile(BVH_file_interface* pyFile);
            MatrixXf m_offsets;
            std::vector<VectorXf> m_rootPositions;
            std::vector<MatrixXf> m_rotations;
            int m_jointNum;
            int m_frameNum;
            float m_frametime;
            std::string m_animName;
            std::string m_modelName;
            std::vector<int> m_topology;
            std::vector<std::string> m_jointNames; // si se quiere usar un subset de las joints originales
            std::vector<VectorXf> m_rootPositions_dmic;
            std::vector<MatrixXf> m_rotations_dmic;
            float m_frametime_dmic;
            static std::string _getFilePath(std::string modelName, std::string animName) {
                std::string relPath = std::string("Assets/CharacterNavigation/") + modelName + "/" + animName + ".bvh";
                std::string filePath = SourcePath(relPath).string();
                return filePath;
            }
        public:
            MatrixXf getOffsets() { return m_offsets; }
            std::vector<VectorXf> getRootPositions() { return m_rootPositions; }
            std::vector<MatrixXf> getRotations() { return m_rotations; }
            int getJointNum() { return m_jointNum; }
            int getFrameNum() { return m_frameNum; }
            float getFrametime() { return m_frametime; }
            std::vector<int> getTopology() { return m_topology;}
            std::vector<std::string> getJointNames() { return m_jointNames; }
            std::string getAnimName() { return m_animName;  }
            std::string getModelName() { return m_modelName; }
            std::vector<VectorXf> getDynamicRootPositions() { return m_rootPositions_dmic; }
            std::vector<MatrixXf> getDynamicRotations() { return m_rotations_dmic; }
            float getDynamicFrametime() { return m_frametime_dmic; }
            void setDynamicData(std::vector<MatrixXf> rotations, std::vector<VectorXf> rootPositions, float frametime);
            std::vector<JointPose> getFramePoses(int frame);
            std::string getInputFilePath() {
                std::string relPath = std::string("Assets/CharacterNavigation/") + m_modelName + "/" + m_animName + ".bvh";
                std::string filePath = SourcePath(relPath).string();
                return filePath;
            }

    };

    class BVHManager {
        private:
            BVHManager() = default;
            std::vector<BVHData*> m_readDataVector;
        public:
            BVHManager& operator=(BVHManager const&) = delete;
            static BVHData* readBVH(std::string modelName, std::string animName);
            static BVHData* readBVH(std::string modelName, std::string animName, std::vector<std::string> jointNames);
            void writeBVHDynamicData(BVHData* data, std::string outAnimName);
            static void StartUp();
            static void ShutDown();
            static BVHManager& GetInstance() noexcept {
                static BVHManager instance;
                return instance;
		    }


    };

}

#endif