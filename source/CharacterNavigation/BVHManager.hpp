#pragma once
#ifndef BVHMANAGER_HPP
#define BVHMANAGER_HPP
#include "bvh_python/cython_interface.h"
#include <string>
#include <vector>
#include <algorithm>

namespace Mona {

    class BVH_file {
        public:
            BVH_file(std::string filePath);
            BVH_file(std::string filePath, std::vector<std::string> jointNames);
            std::vector<int>* m_topology;
            std::vector<std::string>* m_jointNames; // si se quiere usar un subset de las joints originales
            float** m_offsets;
            float** m_rootPositions;
            float*** m_rotations;
            int m_jointNum;
            int m_frameNum;
            float m_frametime;
    private:
        void initFile(BVH_file_interface* pyFile);

    };

    class BVHManager {
        protected:
            BVHManager() {};
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
            static void DestroyInstance();
            bool initialized = false;
            BVH_file* readBVH(std::string filePath);
            BVH_file* readBVH(std::string filePath, std::vector<std::string> jointNames);
            void writeBVH(float*** rotations, float** rootPositions, float frametime, int frameNum, std::string staticDataPath, std::string writePath);


    };

}

#endif