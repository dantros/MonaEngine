#pragma once
#ifndef BVHMANAGER_HPP
#define BVHMANAGER_HPP
#include <string>
#include <vector>
#include "bvh_python/cython_interface.h"
#include <taco.h>

namespace Mona {

	class BVH_file {
        public:
            BVH_file(std::string &filePath);
            BVH_file(std::string& filePath, std::vector<std::string> jointNames);
            std::vector<int>* m_topology;
            std::vector<std::string>* m_jointNames; // si se quiere usar un subset de las joints originales
            taco::Tensor<float>* m_offsets;
            taco::Tensor<float>* m_positions;
            taco::Tensor<float>* m_rotations;
            int m_jointNum;
            int m_frameNum;
            float m_frametime;
    private:
        void initFile(PyObject* pyFile);

    };

    class BVH_writer {
        public:
            BVH_writer(std::string &staticDataPath);
            std::string m_staticDataPath;
            void write(float*** rotations, float** positions, float frametime, int frameNum, std::string& writePath);
    };


}

#endif