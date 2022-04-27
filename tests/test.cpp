#include "./CharacterNavigation/BVHManager.hpp"
#include <iostream>


    
int main(){
    Mona::BVH_file file = Mona::BVH_file::BVH_file(std::string("./BigVegas.bvh"));
    std::cout << file.m_frameNum;
    while (true) {

    }
    return 0;
}

