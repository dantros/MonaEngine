#include "./CharacterNavigation/BVHManager.hpp"
#include <iostream>
#include <algorithm>


    
int main(){
    Mona::BVHManager* bvhManager = Mona::BVHManager::GetInstance();
    std::string file_path = __FILE__;
    std::string dir_path = file_path.substr(0, file_path.rfind("\\"));
    std::replace(dir_path.begin(), dir_path.end(), '\\', '/');
    Mona::BVH_file* file1 = bvhManager->readBVH(dir_path + std::string("/BigVegas.bvh"));
    std::cout << file1->m_frameNum;
    Mona::BVH_file* file2 = bvhManager->readBVH(dir_path + std::string("/BigVegas.bvh"));
    std::cout << file2->m_frameNum;
    Mona::BVHManager::DestroyInstance();
    while (true) {

    }
    return 0;
}

