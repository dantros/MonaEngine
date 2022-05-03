#pragma once
#ifndef BVHMANAGER_HPP
#define BVHMANAGER_HPP
#include "bvh_python/cython_interface.h"
#include <string>
#include <vector>
#include <memory>

namespace Mona {

    struct BVHDynamicData {
        float** rootPositions;
        float*** rotations;
        int jointNum;
        int frameNum;
        float frametime;
    };

    class BVHData {
        private:
            friend class BVHManager;
            friend class IKRig;
            BVHData(std::string filePath);
            BVHData(std::string filePath, std::vector<std::string> jointNames);
            void initFile(BVH_file_interface* pyFile);
            float** m_offsets;
            float** m_rootPositions;
            float*** m_rotations;
            int m_jointNum;
            int m_frameNum;
            float m_frametime;
            std::string m_inputFilePath;
            std::vector<int> m_topology;
            std::vector<std::string> m_jointNames; // si se quiere usar un subset de las joints originales
            BVHDynamicData m_dynamicData;
        public:
            float** getOffsets() { return m_offsets; }
            float** getRootPositions() { return m_rootPositions; }
            float*** getRotations() { return m_rotations; }
            int getJointNum() { return m_jointNum; }
            int getFrameNum() { return m_frameNum; }
            float getFrametime() { return m_frametime; }
            std::vector<int> getTopology() { return m_topology;}
            std::vector<std::string> getJointNames() { return m_jointNames; }
            std::string getInputFilePath() { return m_inputFilePath;  }
            BVHDynamicData getDynamicData() { return m_dynamicData;  }
            void setDynamicData(BVHDynamicData data);
            ~BVHData();
    };

    class BVHManager {
        private:
            BVHManager() = default;
            std::vector<BVHData*> m_readDataVector;
        public:
            BVHManager& operator=(BVHManager const&) = delete;
            static BVHData* readBVH(std::string filePath);
            static BVHData* readBVH(std::string filePath, std::vector<std::string> jointNames);
            void writeBVHDynamicData(BVHData* data, std::string writePath);
            static void StartUp();
            static void ShutDown();
            static BVHManager& GetInstance() noexcept {
                static BVHManager instance;
                return instance;
		    }


    };

}

#endif