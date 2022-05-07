#include "./CharacterNavigation/BVHManager.hpp"
#include <iostream>
#include <algorithm>


    
int main(){
    Mona::BVHManager::StartUp();
    Mona::BVHManager bvhManager = Mona::BVHManager::GetInstance();
    Mona::BVHData* file2 = bvhManager.readBVH("testModel", "input");
    bvhManager.writeBVHDynamicData(file2, "outAnim");
    Mona::BVHManager::ShutDown();
    std::cout << "ready" << std::endl;
    return 0;
}

