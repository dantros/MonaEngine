#include "SkeletonManager.hpp"
#include <stack>
#include "Skeleton.hpp"
#include "../Rendering/Renderer.hpp"
#include "../Core/Log.hpp"
#include "../Core/AssimpTransformations.hpp"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
namespace Mona {
	std::shared_ptr<Skeleton> SkeletonManager::LoadSkeleton(const std::filesystem::path& filePath) noexcept {
		const std::string& stringPath = filePath.string();
		//En caso de que ya exista una entrada en el mapa de esqueletos con el mismo path, 
		// entonces se retorna inmediatamente dicho esqueleto.
		auto it = m_skeletonMap.find(stringPath);
		if (it != m_skeletonMap.end()) {
			return it->second;
		}


		Assimp::Importer importer;
		unsigned int postProcessFlags = aiProcess_Triangulate;
		const aiScene* scene = importer.ReadFile(stringPath, postProcessFlags);
		if (!scene)
		{
			MONA_LOG_ERROR("SkeletonManager Error: Failed to open file with path {0}", stringPath);
			return nullptr;
		}

		//Se llena un mapa con la informacion de todos los huesos
		std::unordered_map<std::string, aiMatrix4x4> boneInfo;
		for (uint32_t i = 0; i < scene->mNumMeshes; i++)
		{
			const aiMesh* meshOBJ = scene->mMeshes[i];
			if (meshOBJ->HasBones()) {
				for (uint32_t j = 0; j < meshOBJ->mNumBones; j++)
				{
					const aiBone* bone = meshOBJ->mBones[j];
					boneInfo.insert(std::make_pair(std::string(bone->mName.C_Str()), bone->mOffsetMatrix));
				}
			}
			
		}

		//Chequeo del tamaño del esqueleto a importar
		if (Renderer::NUM_MAX_BONES < boneInfo.size())
		{
			MONA_LOG_ERROR("SkeletonManager Error: Skeleton at {0} has {1} bones while the engine can only support {2}",
				stringPath,
				boneInfo.size(),
				Renderer::NUM_MAX_BONES);
			return nullptr;
		}
		
		//Declaramos la información necesaria para el esqueto y se reserva la memoria necesaria
		std::vector<glm::mat4> invBindPoseMatrices;
		std::vector<std::string> jointNames;
		std::vector<std::int32_t> parentIndices;
		std::unordered_map<std::string, uint32_t> boneIndexMap;
		invBindPoseMatrices.reserve(boneInfo.size());
		jointNames.reserve(boneInfo.size());
		parentIndices.reserve(boneInfo.size());
		boneIndexMap.reserve(boneInfo.size());
		//El grafo de la escena se reccorre usando DFS (Depth Search First) usando dos stacks. Para poder construir
		// correctamente la jerarquía
		std::stack<int32_t> parentNodeIndices;
		std::stack<const aiNode*> sceneNodes;
		sceneNodes.push(scene->mRootNode);
		parentNodeIndices.push(-1);

		while (!sceneNodes.empty())
		{
			const aiNode* currentNode = sceneNodes.top();
			sceneNodes.pop();
			int32_t parentIndex = parentNodeIndices.top();
			parentNodeIndices.pop();
			//Si el nodo de la escena corresponde a una articulación del esqueleto
			if (boneInfo.find(currentNode->mName.C_Str()) != boneInfo.end())
			{


				aiMatrix4x4& mat = boneInfo[currentNode->mName.C_Str()];
				glm::mat4 m = AssimpToGlmMatrix(mat);
				invBindPoseMatrices.push_back(m);
				jointNames.push_back(currentNode->mName.C_Str());
				parentIndices.push_back(parentIndex);
				boneIndexMap.insert(std::make_pair(currentNode->mName.C_Str(), static_cast<uint32_t>(parentIndices.size() - 1)));
				int32_t newParentIndex = parentIndices.size() - 1;
				for (uint32_t j = 0; j < currentNode->mNumChildren; j++) {
					//Pusheamos los hijos y actualizamos el indice del padre
					sceneNodes.push(currentNode->mChildren[j]);
					parentNodeIndices.push(newParentIndex);
				}
			}

			else {
				for (uint32_t j = 0; j < currentNode->mNumChildren; j++) {
					//Pusheamos los hijos manteniendo el indice del padre
					sceneNodes.push(currentNode->mChildren[j]);
					parentNodeIndices.push(parentIndex);
				}
			}

		}
		Skeleton* skeletonPtr = new Skeleton(std::move(invBindPoseMatrices),
			std::move(jointNames),
			std::move(parentIndices),
			std::move(boneIndexMap));
		std::shared_ptr<Skeleton> skeletonSharedPtr = std::shared_ptr<Skeleton>(skeletonPtr);
		m_skeletonMap.insert({stringPath, skeletonSharedPtr});
		return skeletonSharedPtr;
	}

	void SkeletonManager::CleanUnusedSkeletons() noexcept {
		/*
		* Elimina todos los punteros del mapa cuyo conteo de referencias es igual a uno,
		* es decir, que el puntero del mapa es el unico que apunta a esa memoria.
		*/
		for (auto i = m_skeletonMap.begin(), last = m_skeletonMap.end(); i != last;) {
			if (i->second.use_count() == 1) {
				i = m_skeletonMap.erase(i);
			}
			else {
				++i;
			}

		}
	}

	void SkeletonManager::ShutDown() noexcept {
		//Al cerrar el motor se llama esta función donde se limpia el mapa de equeletos
		m_skeletonMap.clear();
	}
}