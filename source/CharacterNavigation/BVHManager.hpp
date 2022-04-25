#pragma once
#ifndef BVHMANAGER_HPP
#define BVHMANAGER_HPP
#include <string>

namespace Mona {

	class BVH_file {
        public:
            BVH_file(std::string &filePath, std::string* jointNames, std::string* eeNames);
            int* topology;
            int* jointNames;
            std::string eeNames;
            float** offsets;
            float*** positions;
            float*** rotations;
            int jointNum;
            int frameNum;
            int eeNum;
    };

    class BVH_writer {
        public:
            BVH_writer(std::string &staticDataPath);
            void write(float*** rotations, float** positions, std::string& writePath, float frametime);
    };


}

#endif