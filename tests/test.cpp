#include "./CharacterNavigation/BVHManager.hpp"
#include <iostream>
#include <algorithm>


    
int main(){
    std::string file_path = __FILE__;
    std::string dir_path = file_path.substr(0, file_path.rfind("\\"));
    std::replace(dir_path.begin(), dir_path.end(), '\\', '/');
    Mona::BVH_file file = Mona::BVH_file::BVH_file(dir_path + std::string("/BigVegas.bvh"));
    std::cout << file.m_frameNum;
    while (true) {

    }
    return 0;
}

