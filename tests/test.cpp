#include "./CharacterNavigation/BVHManager.hpp"
#include <iostream>
#include <algorithm>


    
int main(){
    Mona::BVHManager::StartUp();
    Mona::BVHManager bvhManager = Mona::BVHManager::GetInstance();
    std::string file_path = __FILE__;
    std::string dir_path = file_path.substr(0, file_path.rfind("\\"));
    std::replace(dir_path.begin(), dir_path.end(), '\\', '/');
    Mona::BVHData* file2 = bvhManager.readBVH("testModel", "input");
    bvhManager.writeBVHDynamicData(file2, "outAnim");
    Mona::BVHManager::ShutDown();
    std::cout << "ready" << std::endl;
    return 0;
}

