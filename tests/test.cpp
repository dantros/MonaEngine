#include "./CharacterNavigation/BVHManager.hpp"
#include <iostream>

namespace Mona{
    
    int main(){
        BVH_file file = BVH_file::BVH_file(const std::string("./BigVegas.bvh"));
        std::cout << file.m_frameNum;
    }
}
