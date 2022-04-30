#pragma once
#ifndef BVHMANAGER_HPP
#define BVHMANAGER_HPP
#include "bvh_python/cython_interface.h"
#include <string>
#include <vector>

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
            void setNewData(float*** rotations, float** rootPositions, int frameNum, float frametime, int frameOffset=0);
    };

    class BVHManager {
        protected:
            BVHManager(){};
            ~BVHManager();
            static BVHManager* singleton;
        public:
            BVHManager(BVHManager& other) = delete;
            void operator=(const BVHManager&) = delete;
            /**
             * This is the static method that controls the access to the singleton
             * instance. On the first run, it creates a singleton object and places it
             * into the static field. On subsequent runs, it returns the client existing
             * object stored in the static field.
             */

            static BVHManager* GetInstance();
            bool initialized = false;
            BVHData readBVH(std::string filePath, bool quater=true);
            BVHData readBVH(std::string filePath, std::vector<std::string> jointNames, bool quater=true);
            void writeBVH(BVHData data, std::string writePath);


    };

}

#endif