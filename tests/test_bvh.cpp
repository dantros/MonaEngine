#include "./CharacterNavigation/BVHManager.hpp"
#include <iostream>
#include <algorithm>


    
int main(){
    Mona::BVHManager::GetInstance().StartUp();
    std::shared_ptr<Mona::BVHData> file = Mona::BVHManager::GetInstance().readBVH("testModel", "input");
    std::cout << "File read" << std::endl;
    Mona::BVHManager::GetInstance().writeBVHDynamicData(file, "outAnim");
    std::cout << "File wrote" << std::endl;
    Mona::BVHManager::GetInstance().ShutDown();
    std::cout << "ready" << std::endl;
    return 0;
}

