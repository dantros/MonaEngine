#include "BVHManager.hpp"
#include <iostream>

namespace Mona{
    
    int main(){
        BVH_file file = BVH_file::BVH_file(std::string("./BigVegas.bvh"));
        std::cout << file.m_frameNum;
    }
}
