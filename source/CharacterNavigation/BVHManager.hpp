#pragma once
#ifndef BVHMANAGER_HPP
#define BVHMANAGER_HPP
#include <string>

namespace Mona {

	class BVH_file {
        public:
            BVH_file(std::string &filePath, std::string* jointNames, std::string* eeNames);
            int* m_topology;
            std::string* m_jointNames;
            std::string* m_eeNames;
            float** m_offsets;
            float*** m_positions;
            float*** m_rotations;
            int m_jointNum;
            int m_frameNum;
            int m_eeNum;
            float m_frametime;	

    };

    class BVH_writer {
        public:
            BVH_writer(std::string &staticDataPath);
            std::string m_staticDataPath;
            void write(float*** rotations, float** positions, float frametime, std::string& writePath);
    };


}

#endif