#include "./CharacterNavigation/BVHManager.hpp"
#include <iostream>
#include <algorithm>


    
int main(){
    Mona::BVHManager* bvhManager = Mona::BVHManager::GetInstance();
    std::string file_path = __FILE__;
    std::string dir_path = file_path.substr(0, file_path.rfind("\\"));
    std::replace(dir_path.begin(), dir_path.end(), '\\', '/');
    Mona::BVHData file1 = bvhManager->readBVH(dir_path + std::string("/input.bvh"), false);
    std::string writePath1 = dir_path + std::string("/euler_result.bvh");
    Mona::BVHData file2 = bvhManager->readBVH(dir_path + std::string("/input.bvh"), true);
    std::string writePath2 = dir_path + std::string("/quater_result.bvh");
    bvhManager->writeBVH(file1, writePath1);
    bvhManager->writeBVH(file2, writePath2);
    std::cout << "ready";
    return 0;
}

