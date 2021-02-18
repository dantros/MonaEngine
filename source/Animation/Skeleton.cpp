#include "Skeleton.hpp"
#include <stack>
#include "Skeleton.hpp"
#include "../Rendering/Renderer.hpp"
#include "../Core/Log.hpp"
#include "../Core/AssimpTransformations.hpp"
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
namespace Mona {

	Skeleton::Skeleton(const std::string& filePath) {
		Assimp::Importer importer;
		unsigned int postProcessFlags = aiProcess_Triangulate;
		const aiScene* scene = importer.ReadFile(filePath, postProcessFlags);
		if (!scene)
		{
			MONA_LOG_ERROR("Skeleton Error: Failed to open file with path {0}", filePath);
			return;
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
			MONA_LOG_ERROR("Skeleton Error: Skeleton at {0} has {1} bones while the engine can only support {2}",
				filePath,
				boneInfo.size(),
				Renderer::NUM_MAX_BONES);
			return;
		}

		//Se reserva la memoria necesaria
		m_invBindPoseMatrices.reserve(boneInfo.size());
		m_jointNames.reserve(boneInfo.size());
		m_parentIndices.reserve(boneInfo.size());
		m_jointMap.reserve(boneInfo.size());
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
				m_invBindPoseMatrices.push_back(m);
				m_jointNames.push_back(currentNode->mName.C_Str());
				m_parentIndices.push_back(parentIndex);
				m_jointMap.insert(std::make_pair(currentNode->mName.C_Str(), static_cast<uint32_t>(m_parentIndices.size() - 1)));
				int32_t newParentIndex = m_parentIndices.size() - 1;
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

	}
	
}
