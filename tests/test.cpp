#include "./CharacterNavigation/BVHManager.hpp"
#include <iostream>
#include <algorithm>


    
int main(){
    std::string file_path = __FILE__;
    std::string dir_path = file_path.substr(0, file_path.rfind("\\"));
    std::replace(dir_path.begin(), dir_path.end(), '\\', '/');
    Mona::BVH_file file = Mona::BVH_file::BVH_file(dir_path + std::string("/BigVegas.bvh"));
    for (int i = 0; i < file.m_frameNum; i++) {
        std::cout << "frame " << i;
        for (int j = 0; j < file.m_jointNum; j++) {
            std::cout << "joint " << j << ": [" << file.m_rotations[i][j][0] << ", " << file.m_rotations[i][j][1] << ", " << file.m_rotations[i][j][2] << "]";
        }
    }
    while (true) {

    }
    return 0;
}

