#include "SkinnedMesh.hpp"

#include "../Core/Log.hpp"
#include "../Core/AssimpTransformations.hpp"
#include <glm/glm.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <vector>
#include <stack>
#include <glad/glad.h>
#include "Skeleton.hpp"
namespace Mona {

	struct SkeletalMeshVertex {
		glm::vec3 position;
		glm::vec3 normal;
		glm::vec2 uv;
		glm::vec3 tangent;
		glm::vec3 bitangent;
		glm::vec4 boneIds;
		glm::vec4 boneWeights;

	};

	SkinnedMesh::~SkinnedMesh() {
		if (m_vertexArrayID)
			ClearData();
	}

	void SkinnedMesh::ClearData() noexcept {
		MONA_ASSERT(m_vertexArrayID, "SkinnedMesh Error: Trying to delete already deleted mesh");
		MONA_ASSERT(m_vertexBufferID,"SkinnedMesh Error: Trying to delete already deleted mesh");
		MONA_ASSERT(m_indexBufferID, "SkinnedMesh Error: Trying to delete already deleted mesh");
		glDeleteBuffers(1, &m_vertexBufferID);
		glDeleteBuffers(1, &m_indexBufferID);
		glDeleteVertexArrays(1, &m_vertexArrayID);
		m_vertexArrayID = 0;
	}

	SkinnedMesh::SkinnedMesh(std::shared_ptr<Skeleton> skeleton,
		const std::string& filePath,
		bool flipUvs) : 
		m_vertexArrayID(0),
		m_vertexBufferID(0),
		m_indexBufferID(0),
		m_indexBufferCount(0),
		m_skeletonPtr(skeleton)
	{
		MONA_ASSERT(skeleton != nullptr, "SkinnedMesh Error: Skeleton cannot be null");
		Assimp::Importer importer;
		unsigned int postProcessFlags = flipUvs ? aiProcess_FlipUVs : 0;
		postProcessFlags |= aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_GenUVCoords | aiProcess_CalcTangentSpace;
		const aiScene* scene = importer.ReadFile(filePath, postProcessFlags);



		if (!scene) {
			MONA_LOG_ERROR("SkinnedMesh Error: Failed to open file with path {0}", filePath);
			return;
		}

		//Comienzo del proceso de pasar desde la escena de assimp a un formato interno
		std::vector<SkeletalMeshVertex> vertices;
		std::vector<unsigned int> faces;
		size_t numVertices = 0;
		size_t numFaces = 0;
		//El primer paso consiste en contar el numero de vertices y caras totales
		//de esta manera se puede reservar memoria inmediatamente evitando realocación de memoria
		for (uint32_t i = 0; i < scene->mNumMeshes; i++) {
			numVertices += scene->mMeshes[i]->mNumVertices;
			numFaces += scene->mMeshes[i]->mNumFaces;
		}

		vertices.reserve(numVertices);
		faces.reserve(numFaces);

		//El grafo de la escena se reccorre usando DFS (Depth Search First) usando dos stacks.
		std::stack<const aiNode*> sceneNodes;
		std::stack<aiMatrix4x4> sceneTransforms;
		std::unordered_map<std::string, aiMatrix4x4> boneInfo;

		//Luego pusheamos información asociada a la raiz del grafo
		sceneNodes.push(scene->mRootNode);
		sceneTransforms.push(scene->mRootNode->mTransformation);
		unsigned int offset = 0;
		while (!sceneNodes.empty())
		{
			const aiNode* currentNode = sceneNodes.top();
			const auto currentTransform = sceneTransforms.top();
			auto currentInvTranspose = currentTransform;
			//Las normales transforman distinto que las posiciones
			//Por eso es necesario invertir la matrix y luego trasponerla
			//Ver: https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/geometry/transforming-normals
			currentInvTranspose.Inverse().Transpose();
			sceneNodes.pop();
			sceneTransforms.pop();
			for (uint32_t j = 0; j < currentNode->mNumMeshes; j++) {
				const aiMesh* meshOBJ = scene->mMeshes[currentNode->mMeshes[j]];
				for (uint32_t i = 0; i < meshOBJ->mNumVertices; i++) {
					aiVector3D position = currentTransform * meshOBJ->mVertices[i];
					aiVector3D tangent = currentTransform * meshOBJ->mTangents[i];
					tangent.Normalize();
					aiVector3D normal = currentInvTranspose * meshOBJ->mNormals[i];
					normal.Normalize();
					aiVector3D bitangent = currentTransform * meshOBJ->mBitangents[i];
					bitangent.Normalize();

					SkeletalMeshVertex vertex;
					vertex.position = AssimpToGlmVec3(position);
					vertex.normal = AssimpToGlmVec3(normal);
					vertex.tangent = AssimpToGlmVec3(tangent);
					vertex.bitangent = AssimpToGlmVec3(bitangent);
					if (meshOBJ->mTextureCoords[0]) {
						vertex.uv.x = meshOBJ->mTextureCoords[0][i].x;
						vertex.uv.y = meshOBJ->mTextureCoords[0][i].y;
					}
					else {
						vertex.uv = glm::vec2(0.0f);
					}
					vertex.boneIds = glm::ivec4(0);
					vertex.boneWeights = glm::vec4(0.0f);

					vertices.push_back(vertex);
				}

				for (uint32_t i = 0; i < meshOBJ->mNumFaces; i++) {
					const aiFace& face = meshOBJ->mFaces[i];
					MONA_ASSERT(face.mNumIndices == 3, "SkinnedMesh Error: Can load meshes with faces that have {0} vertices", face.mNumIndices);
					faces.push_back(face.mIndices[0] + offset);
					faces.push_back(face.mIndices[1] + offset);
					faces.push_back(face.mIndices[2] + offset);
				}

				for (uint32_t i = 0; i < meshOBJ->mNumBones; i++)
				{
					const aiBone* bone = meshOBJ->mBones[i];
					boneInfo.insert(std::make_pair(std::string(bone->mName.C_Str()), bone->mOffsetMatrix));
				}
				offset += meshOBJ->mNumVertices;


			}




			for (uint32_t j = 0; j < currentNode->mNumChildren; j++) {
				//Pusheamos los hijos y acumulamos la matrix de transformación
				sceneNodes.push(currentNode->mChildren[j]);
				sceneTransforms.push(currentNode->mChildren[j]->mTransformation * currentTransform);
			}
		}

		//Recorremos la escena nuevamente ahora para llenar los datos de la piel del la malla
		sceneNodes.push(scene->mRootNode);
		std::vector<uint32_t> boneCounts;
		boneCounts.resize(numVertices);
		uint32_t vertexOffset = 0;

		while (!sceneNodes.empty())
		{
			const aiNode* currentNode = sceneNodes.top();
			sceneNodes.pop();
			for (uint32_t j = 0; j < currentNode->mNumMeshes; j++) {
				const aiMesh* meshOBJ = scene->mMeshes[currentNode->mMeshes[j]];
				for (uint32_t i = 0; i < meshOBJ->mNumBones; i++)
				{
					const aiBone* bone = meshOBJ->mBones[i];
					int32_t signIndex = skeleton->GetJointIndex(bone->mName.C_Str());
					MONA_ASSERT(signIndex >= 0, "Skinned Error: Given skeleton incompatible with mesh being imported");
					uint32_t index = static_cast<uint32_t>(signIndex);
					for (uint32_t k = 0; k < bone->mNumWeights; k++)
					{
						uint32_t id = bone->mWeights[k].mVertexId + vertexOffset;
						float weight = bone->mWeights[k].mWeight;
						boneCounts[id]++;
						switch (boneCounts[id]) {
						case 1:
							vertices[id].boneIds.x = index;
							vertices[id].boneWeights.x = weight;
							break;
						case 2:
							vertices[id].boneIds.y = index;
							vertices[id].boneWeights.y = weight;
							break;
						case 3:
							vertices[id].boneIds.z = index;
							vertices[id].boneWeights.z = weight;
							break;
						case 4:
							vertices[id].boneIds.w = index;
							vertices[id].boneWeights.w = weight;
							break;
						default:
							MONA_LOG_INFO("SkinnedMesh Info: Engine only supports a maximun of 4 bones per vertex.");
							break;

						}
					}

				}
				vertexOffset += meshOBJ->mNumVertices;

			}
			for (uint32_t j = 0; j < currentNode->mNumChildren; j++) {
				//Pusheamos los hijos
				sceneNodes.push(currentNode->mChildren[j]);
			}
		}


		//Normalizamos los pesos de los vertices
		for (int i = 0; i < vertices.size(); i++) {
			glm::vec4& boneWeights = vertices[i].boneWeights;
			float totalWeight = boneWeights.x + boneWeights.y + boneWeights.z + boneWeights.w;
			if (totalWeight > 0.0f) {
				vertices[i].boneWeights = glm::vec4(
					boneWeights.x / totalWeight,
					boneWeights.y / totalWeight,
					boneWeights.z / totalWeight,
					boneWeights.w / totalWeight
				);
			}
			else {
				MONA_ASSERT(false, "SkinnedMesh Error: Vertex with all bone weights equal to 0.");

			}
		}
		//Comienza el paso de los datos en CPU a GPU usando OpenGL
		m_indexBufferCount = static_cast<uint32_t>(faces.size());
		glGenVertexArrays(1, &m_vertexArrayID);
		glBindVertexArray(m_vertexArrayID);

		glGenBuffers(1, &m_vertexBufferID);
		glGenBuffers(1, &m_indexBufferID);
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferID);
		glBufferData(GL_ARRAY_BUFFER, static_cast<unsigned int>(vertices.size()) * sizeof(SkeletalMeshVertex), vertices.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBufferID);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<unsigned int>(faces.size()) * sizeof(unsigned int), faces.data(), GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(SkeletalMeshVertex), (void*)offsetof(SkeletalMeshVertex, position));
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(SkeletalMeshVertex), (void*)offsetof(SkeletalMeshVertex, normal));
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(SkeletalMeshVertex), (void*)offsetof(SkeletalMeshVertex, uv));
		glEnableVertexAttribArray(3);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(SkeletalMeshVertex), (void*)offsetof(SkeletalMeshVertex, tangent));
		glEnableVertexAttribArray(4);
		glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(SkeletalMeshVertex), (void*)offsetof(SkeletalMeshVertex, bitangent));
		glEnableVertexAttribArray(5);
		glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(SkeletalMeshVertex), (void*)offsetof(SkeletalMeshVertex, boneIds));
		glEnableVertexAttribArray(6);
		glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(SkeletalMeshVertex), (void*)offsetof(SkeletalMeshVertex, boneWeights));


	}
}
