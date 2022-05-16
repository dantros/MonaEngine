#include "./CharacterNavigation/BVHManager.hpp"
#include <iostream>
#include <algorithm>


    
int main(){
    Mona::BVHManager::StartUp();
    Mona::BVHManager bvhManager = Mona::BVHManager::GetInstance();
    std::vector<std::string> jointNames = { "Pelvis", "LeftUpLeg", "LeftLeg", "Hips" };
    Mona::BVHData* file = bvhManager.readBVH("testModel", "input", jointNames);
    //Mona::BVHData* file = bvhManager.readBVH("testModel", "input");
    bvhManager.writeBVHDynamicData(file, "outAnim");
    Mona::BVHManager::ShutDown();
    std::cout << "ready" << std::endl;
    return 0;
}

