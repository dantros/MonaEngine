#pragma once
#ifndef BVHMANAGER_HPP
#define BVHMANAGER_HPP
#include <string>
#include <vector>
#include "bvh_python/cython_interface.h"
#include <unsupported/Eigen/CXX11/Tensor>

namespace Mona {

	class BVH_file {
        public:
            BVH_file(std::string filePath);
            BVH_file(std::string filePath, std::vector<std::string> jointNames);
            std::vector<int>* m_topology;
            std::vector<std::string>* m_jointNames; // si se quiere usar un subset de las joints originales
            Eigen::Tensor<float, 2> m_offsets;
            Eigen::Tensor<float, 3> m_positions;
            Eigen::Tensor<float, 3> m_rotations;
            int m_jointNum;
            int m_frameNum;
            float m_frametime;
    private:
        void initFile(BVH_file_interface pyFile);

    };

    class BVH_writer {
        public:
            BVH_writer(std::string staticDataPath);
            std::string m_staticDataPath;
            void write(float*** rotations, float** positions, float frametime, int frameNum, std::string writePath);
    };


}

#endif