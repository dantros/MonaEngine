#include "./CharacterNavigation/BVHManager.hpp"
#include <iostream>
#include <algorithm>


    
int main(){
    Mona::BVHManager::GetInstance().StartUp();
    std::shared_ptr<Mona::BVHData> file = Mona::BVHManager::GetInstance().readBVH("testModel", "akaiIn");
    std::cout << "File read" << std::endl;
    auto rot = file->getRotations();
    auto pos = file->getRootPositions();
    for (int i = 0; i < rot.size(); i++) {
        for (int j = 0; j < rot[i].size(); j++) {
            rot[i][j].setIdentity();
        }
    }
    for (int i = 0; i < pos.size(); i++) {
         pos[i] = { 0,0,0 };
        
    }
    file->setDynamicData(rot, pos, file->getFrametime());
    Mona::BVHManager::GetInstance().writeBVHDynamicData(file, "akaiOut");
    std::cout << "File wrote" << std::endl;
    Mona::BVHManager::GetInstance().ShutDown();
    std::cout << "ready" << std::endl;
    return 0;
}

