#pragma once
#ifndef BVHMANAGER_HPP
#define BVHMANAGER_HPP
#include "bvh_python/cython_interface.h"
#include <string>
#include <vector>
#include <memory>

namespace Mona {

    class BVHData {
        private:
            friend class BVHManager;
            BVHData(std::string filePath, bool quater = true);
            BVHData(std::string filePath, std::vector<std::string> jointNames, bool quater = true);
            void initFile(BVH_file_interface* pyFile);
            float** m_offsets;
            float** m_rootPositions;
            float*** m_rotations;
            int m_jointNum;
            int m_frameNum;
            int m_frameOffset = 0; // indica el frame en que se insertaria la animacion de esta BVHData
            float m_frametime;
            bool m_quater;
            std::string m_inputFilePath;
            std::vector<int>* m_topology;
            std::vector<std::string>* m_jointNames; // si se quiere usar un subset de las joints originales
        public:
            float** getOffsets() { return m_offsets; }
            float** getRootPositions() { return m_rootPositions; }
            float*** getRotations() { return m_rotations; }
            int getJointNum() { return m_jointNum; }
            int getFrameNum() { return m_frameNum; }
            int getFrameOffset() { return m_frameOffset; }
            float getFrametime() { return m_frametime; }
            std::vector<int>* getTopology() { return m_topology;}
            std::vector<std::string>* getJointNames() { return m_jointNames; }
            std::string getInputFilePath() { return m_inputFilePath;  }
            bool isQuater() { return m_quater; }
            void setNewData(float*** rotations, float** rootPositions, int frameNum, float frametime, bool quater=true, int frameOffset=0);
    };

    class BVHManager {
        private:
            BVHManager() = default;
        public:
            BVHManager& operator=(BVHManager const&) = delete;
            static bool initialized;
            static bool exited;
            BVHData readBVH(std::string filePath, bool quater=true);
            BVHData readBVH(std::string filePath, std::vector<std::string> jointNames, bool quater=true);
            void writeBVH(BVHData data, std::string writePath);
            static void StartUp();
            static void ShutDown();
            static BVHManager& GetInstance() noexcept {
                static BVHManager instance;
                return instance;
		    }


    };

}

#endif