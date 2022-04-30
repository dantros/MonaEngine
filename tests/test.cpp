#include "./CharacterNavigation/BVHManager.hpp"
#include <iostream>
#include <algorithm>


    
int main(){
    Mona::BVHManager* bvhManager = Mona::BVHManager::GetInstance();
    std::string file_path = __FILE__;
    std::string dir_path = file_path.substr(0, file_path.rfind("\\"));
    std::replace(dir_path.begin(), dir_path.end(), '\\', '/');
    Mona::BVHData file1 = bvhManager->readBVH(dir_path + std::string("/input.bvh"));
    std::string staticPath = dir_path + std::string("/input.bvh");
    std::string writePath = dir_path + std::string("/inputCopy.bvh");
    bvhManager->writeBVH(file1.getRotations(), file1.getRootPositions(), file1.getFrametime(), file1.getFrameNum(), staticPath, writePath);
    while (true) {

    }
    return 0;
}

