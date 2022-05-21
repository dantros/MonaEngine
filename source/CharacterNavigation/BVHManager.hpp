#pragma once
#ifndef BVHMANAGER_HPP
#define BVHMANAGER_HPP
#include "bvh_python/cython_interface.h"
#include <string>
#include <vector>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include "../Animation/AnimationController.hpp"
#include "../Core/RootDirectory.hpp"

namespace Mona {
    typedef Eigen::Matrix<float, 1, 3> Vector3f;
    typedef Eigen::Quaternion<float> Quaternion;

    class BVHData {
        private:
            friend class BVHManager;
            BVHData(std::shared_ptr<AnimationClip> animation);
            BVHData(std::string modelName, std::string animName);
            void initFile(BVH_file_interface* pyFile);
            std::vector<Vector3f> m_offsets; //numJoints x 3
            std::vector<Vector3f> m_rootPositions; // frameNum x 3
            std::vector<std::vector<Quaternion>> m_rotations; // frameNum x numJoints x 4
            int m_jointNum;
            int m_frameNum;
            float m_frametime;
            std::string m_animName;
            std::string m_modelName;
            std::vector<int> m_topology;
            std::vector<std::string> m_jointNames;
            std::vector<Vector3f> m_rootPositions_dmic;
            std::vector<std::vector<Quaternion>> m_rotations_dmic;
            float m_frametime_dmic;
            std::shared_ptr<AnimationClip> m_baseClip = nullptr;
            static std::string _getFilePath(std::string modelName, std::string animName) {
                std::string relPath = std::string("Assets/CharacterNavigation/") + modelName + "/" + animName + ".bvh";
                std::string filePath = SourcePath(relPath).string();
                return filePath;
            }
        public:
            std::vector<Vector3f> getOffsets() { return m_offsets; }
            std::vector<Vector3f> getRootPositions() { return m_rootPositions; }
            std::vector<std::vector<Quaternion>> getRotations() { return m_rotations; }
            int getJointNum() { return m_jointNum; }
            int getFrameNum() { return m_frameNum; }
            float getFrametime() { return m_frametime; }
            std::vector<int> getTopology() { return m_topology;}
            std::vector<std::string> getJointNames() { return m_jointNames; }
            std::string getAnimName() { return m_animName;  }
            std::string getModelName() { return m_modelName; }
            std::vector<Vector3f> getDynamicRootPositions() { return m_rootPositions_dmic; }
            std::vector<std::vector<Quaternion>> getDynamicRotations() { return m_rotations_dmic; }
            float getDynamicFrametime() { return m_frametime_dmic; }
            void setDynamicData(std::vector<std::vector<Quaternion>> rotations, std::vector<Vector3f> rootPositions, float frametime);
            std::vector<JointPose> getFramePoses(int frame);
            std::string getInputFilePath() {
                std::string relPath = std::string("Assets/CharacterNavigation/") + m_modelName + "/" + m_animName + ".bvh";
                std::string filePath = SourcePath(relPath).string();
                return filePath;
            }
            std::shared_ptr<AnimationClip> getBaseAnimationClip() { return m_baseClip; }

    };

    struct strPairHash {
    public:
        std::size_t operator()(const std::pair<std::string, std::string>& x) const
        {   
            size_t hash1 = std::hash<std::string>()(x.first);
            size_t hash2 = std::hash<std::string>()(x.second);
            size_t dualHash = hash1<<1 + hash1 + hash2;
            return dualHash;
        }
    };
    class BVHManager {
        friend class World;
        public:
            using BVHDataMap = std::unordered_map<std::pair<std::string, std::string>, std::shared_ptr<BVHData>, strPairHash>;
            BVHManager(BVHManager const&) = delete;
            BVHManager& operator=(BVHManager const&) = delete;
            std::shared_ptr <BVHData> readBVH(std::shared_ptr<AnimationClip> animation);
            std::shared_ptr <BVHData> readBVH(std::string modelName, std::string animName);
            std::shared_ptr <BVHData> getBVHData(std::shared_ptr<AnimationClip> animation);
            std::shared_ptr<BVHData> getBVHData(std::string modelName, std::string animName);
            void writeBVHDynamicData(std::shared_ptr <BVHData> data, std::string outAnimName);
            void CleanUnusedBVHClips() noexcept;
            void StartUp();
            void ShutDown();
            static BVHManager& GetInstance() noexcept {
                static BVHManager instance;
                return instance;
		    }
    private:
        BVHManager() = default;
        BVHDataMap m_bvhDataMap;


    };

}

#endif