#include "MeshManager.hpp"
#include <cstdint>
#include <vector>
#include <stack>
#include <cmath>
#include <glm/gtc/constants.hpp>
#include <glm/ext/matrix_relational.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include "../Core/Log.hpp"
#include "Mesh.hpp"
#include "../Core/RootDirectory.hpp"
#include "../Animation/Skeleton.hpp"
#include "../Animation/AnimationClip.hpp"
#include <glad/glad.h>

namespace Mona {
	
	struct MeshVertex {
		glm::vec3 position;
		glm::vec3 normal;
		glm::vec2 uv;
		glm::vec3 tangent;
		glm::vec3 bitangent;
	};

	struct SkeletalMeshVertex {
		glm::vec3 position;
		glm::vec3 normal;
		glm::vec2 uv;
		glm::vec3 tangent;
		glm::vec3 bitangent;
		glm::vec4 boneIds;
		glm::vec4 boneWeights;

	};

	glm::mat4 AssimpToGlmMatrix(const aiMatrix4x4& mat)
	{
		glm::mat4 m;
		for (int y = 0; y < 4; y++)
		{
			for (int x = 0; x < 4; x++)
			{
				m[x][y] = mat[y][x];
			}
		}
		return m;
	}

	glm::vec3 AssimpToGlmVec3(const aiVector3D&vec)
	{
		return glm::vec3(vec.x, vec.y, vec.z);
	}

	glm::fquat AssimpToGlmQuat(const aiQuaternion& quat) {
		glm::fquat q;
		q.x = quat.x;
		q.y = quat.y;
		q.z = quat.z;
		q.w = quat.w;

		return q;
	}

	std::shared_ptr<Mesh> MeshManager::LoadMesh(PrimitiveType type) noexcept
	{
		switch (type)
		{
			case Mona::MeshManager::PrimitiveType::Plane:
			{
				auto it = m_meshMap.find("Plane");
				if (it != m_meshMap.end())
				{
					return it->second;
				}
				return LoadPlane();
				break;
			}
			case Mona::MeshManager::PrimitiveType::Cube:
			{
				auto it = m_meshMap.find("Cube");
				if (it != m_meshMap.end())
				{
					return it->second;
				}
				return LoadCube();
				break;
			}
			case Mona::MeshManager::PrimitiveType::Sphere:
			{
				auto it = m_meshMap.find("Sphere");
				if (it != m_meshMap.end())
				{
					return it->second;
				}
				return LoadSphere();
				break;
			}
			default:
			{
				auto it = m_meshMap.find("Sphere");
				if (it != m_meshMap.end())
				{
					return it->second;
				}
				return LoadSphere();
				break;
			}
		}
	}

	std::shared_ptr<Mesh> MeshManager::LoadMesh(const std::filesystem::path& filePath, bool flipUVs) noexcept {
		const std::string& stringPath = filePath.string();
		//En caso de que ya exista una entrada en el mapa de mallas con el mismo path, entonces se retorna inmediatamente
		//dicha malla.
		auto it = m_meshMap.find(stringPath);
		if (it != m_meshMap.end()) {
			return it->second;
		}

		//En caso contrario comienza el proceso de importación
		//El primer caso consiste en cargar la escena del archivo ubicada en filepath usando assimp
		Assimp::Importer importer;
		unsigned int postProcessFlags = flipUVs ? aiProcess_FlipUVs : 0;
		postProcessFlags |= aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_GenUVCoords | aiProcess_CalcTangentSpace;
		const aiScene* scene = importer.ReadFile(stringPath, postProcessFlags);

		

		if (!scene) {
			//En caso de fallar la carga se envia un mensaje de error y se carga un cubo.
			MONA_LOG_ERROR("MeshManager Error: Failed to open file with path {0}", stringPath);
			MONA_LOG_INFO("Loading default model");
			return LoadMesh(PrimitiveType::Cube);
		}

		//Comienzo del proceso de pasar desde la escena de assimp a un formato interno
		std::vector<MeshVertex> vertices;
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
		//Luego pusheamos información asociada a la raiz del grafo
		sceneNodes.push(scene->mRootNode);
		sceneTransforms.push(scene->mRootNode->mTransformation);
		unsigned int offset = 0;
		while (!sceneNodes.empty())
		{
			const aiNode* currentNode = sceneNodes.top();
			auto currentTransform = sceneTransforms.top();
			sceneNodes.pop();
			sceneTransforms.pop();
			auto currentInvTranspose = currentTransform;
			//Las normales transforman distinto que las posiciones
			//Por eso es necesario invertir la matrix y luego trasponerla
			//Ver: https://www.scratchapixel.com/lessons/mathematics-physics-for-computer-graphics/geometry/transforming-normals
			currentInvTranspose.Inverse().Transpose();
			
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
					
					MeshVertex vertex;
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
					vertices.push_back(vertex);

				}

				for (uint32_t i = 0; i < meshOBJ->mNumFaces; i++) {
					const aiFace& face = meshOBJ->mFaces[i];
					MONA_ASSERT(face.mNumIndices == 3, "Model Manager Error: Can load meshes with faces that have {0} vertices", face.mNumIndices);
					faces.push_back(face.mIndices[0] + offset);
					faces.push_back(face.mIndices[1] + offset);
					faces.push_back(face.mIndices[2] + offset);
				}
				offset += meshOBJ->mNumVertices;


			}
			
			for (uint32_t j = 0; j < currentNode->mNumChildren; j++) {
				//Pusheamos los hijos y acumulamos la matrix de transformación
				sceneNodes.push(currentNode->mChildren[j]);
				sceneTransforms.push(currentNode->mChildren[j]->mTransformation * currentTransform);
			}
		}
		
		//Comienza el paso de los datos en CPU a GPU usando OpenGL
		unsigned int modelVBO, modelIBO, modelVAO;
		glGenVertexArrays(1, &modelVAO);
		glBindVertexArray(modelVAO);

		glGenBuffers(1, &modelVBO);
		glGenBuffers(1, &modelIBO);
		glBindBuffer(GL_ARRAY_BUFFER, modelVBO);
		glBufferData(GL_ARRAY_BUFFER, static_cast<unsigned int>(vertices.size()) * sizeof(MeshVertex), vertices.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, modelIBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<unsigned int>(faces.size()) * sizeof(unsigned int), faces.data(), GL_STATIC_DRAW);
		//Un vertice de la malla se ve como
		// v = {pos_x, pos_y, pos_z, normal_x, normal_y, normal_z, uv_u, uv_v, tangent_x, tangent_y, tangent_z}
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), (void*)offsetof(MeshVertex, position));
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), (void*)offsetof(MeshVertex, normal));
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), (void*)offsetof(MeshVertex, uv));
		glEnableVertexAttribArray(3);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), (void*)offsetof(MeshVertex, tangent));
		glEnableVertexAttribArray(4);
		glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), (void*)offsetof(MeshVertex, bitangent));
		Mesh* meshPtr = new Mesh(modelVAO, modelVBO, modelIBO, static_cast<uint32_t>(faces.size()));
		std::shared_ptr<Mesh> sharedPtr = std::shared_ptr<Mesh>(meshPtr);
		//Antes de retornar la malla recien cargada, insertamos esta al mapa para que cargas futuras sean mucho mas rapidas.
		m_meshMap.insert({ stringPath, sharedPtr });
		return sharedPtr;

	}

	std::shared_ptr<Mesh> MeshManager::LoadSphere() noexcept {
		//Esta implementación de la creacion procedural de la malla de una esfera
		//esta basada en: http://www.songho.ca/opengl/gl_sphere.html

		//Cada vertice debe tener la forma
		// v = {p_x, p_y, p_z, n_x, n_y, n_z, uv_u, uv_v, t_x, t_y, t_z, b_x, b_y, b_z};
		std::vector<float> vertices;
		std::vector<unsigned int> indices;
		unsigned int stackCount = 16;
		unsigned int sectorCount = 32;
		constexpr float PI = glm::pi<float>();
		float sectorStep = 2 * PI / sectorCount;
		float stackStep = PI / stackCount;
		float sectorAngle, stackAngle;
		float radius = 1.0f;

		float x, y, z;
		// Coordenadas esfericas
		// x = cos(pi/2 - stackAngle) * cos(sectorAngle)
		// y = cos(pi/2 - stackAngle) * sin(sectorAngle)
		// z = sin(pi/2 - stackAngle)
		for (unsigned int i = 0; i <= stackCount; i++)
		{
			stackAngle = PI / 2.0f - i * stackStep;
			float cosStackAngle = std::cos(stackAngle);
			float sinStackAngle = std::sin(stackAngle);      
			z = sinStackAngle;
			for (unsigned int j = 0; j <= sectorCount; j++)
			{
				sectorAngle = sectorStep * j;
				float cosSectorAngle = std::cos(sectorAngle);
				float sinSectorAngle = std::sin(sectorAngle);
				x = cosStackAngle * cosSectorAngle;
				y = cosStackAngle * sinSectorAngle;

				vertices.push_back(radius * x);
				vertices.push_back(radius * y);
				vertices.push_back(radius * z);
				vertices.push_back(x);
				vertices.push_back(y);
				vertices.push_back(z);
				float u = (float)j / (float) sectorCount;
				float v = (float)i / (float) stackCount;
				vertices.push_back(u);
				vertices.push_back(v);
				//Tangent dr/dSectorAngle
				float tx = -sinSectorAngle;
				float ty = cosSectorAngle;
				float tz = 0.0f;
				vertices.push_back(tx);
				vertices.push_back(ty);
				vertices.push_back(tz);
				//Bitangent dr/dStackAngle
				float bx = -sinStackAngle * cosSectorAngle;
				float by = -sinStackAngle * sinSectorAngle;
				float bz = cosStackAngle;
				vertices.push_back(bx);
				vertices.push_back(by);
				vertices.push_back(bz);
			}

		}

		unsigned int k1, k2;
		for (unsigned int i = 0; i < stackCount; ++i)
		{
			k1 = i * (sectorCount + 1);    
			k2 = k1 + sectorCount + 1;   

			for (unsigned int j = 0; j < sectorCount; ++j, ++k1, ++k2)
			{
				if (i != 0)
				{
					indices.push_back(k1);
					indices.push_back(k2);
					indices.push_back(k1 + 1);
				}

				if (i != (stackCount - 1))
				{
					indices.push_back(k1 + 1);
					indices.push_back(k2);
					indices.push_back(k2 + 1);
				}
			}
		}
		unsigned int sphereVBO, sphereIBO, sphereVAO;
		glGenVertexArrays(1, &sphereVAO);
		glBindVertexArray(sphereVAO);

		glGenBuffers(1, &sphereVBO);
		glGenBuffers(1, &sphereIBO);
		glBindBuffer(GL_ARRAY_BUFFER, sphereVBO);
		glBufferData(GL_ARRAY_BUFFER, static_cast<unsigned int>(vertices.size())*sizeof(float), vertices.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sphereIBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<unsigned int>(indices.size()) * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(6 * sizeof(float)));
		glEnableVertexAttribArray(3);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(8 * sizeof(float)));
		glEnableVertexAttribArray(4);
		glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(11 * sizeof(float)));
		Mesh* meshPtr = new Mesh(sphereVAO, sphereVBO, sphereIBO, static_cast<uint32_t>(indices.size()));
		std::shared_ptr<Mesh> sharedPtr = std::shared_ptr<Mesh>(meshPtr);
		m_meshMap.insert({ "Sphere", sharedPtr });
		return sharedPtr;
	}

	std::shared_ptr<Mesh> MeshManager::LoadCube() noexcept {
		// Cada vertice tiene la siguiente forma
		// v = {p_x, p_y, p_z, n_x, n_y, n_z, uv_u, uv_v, t_x, t_y, t_z, b_x, b_y, b_z};
		float vertices[] = {
	   -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f,
		1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f,
		1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f,
		1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f,
	   -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f,
	   -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, -1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f,

	   -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f,  0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
		1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f,  0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
		1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f,  1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
		1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 1.0f,  1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
	   -1.0f,  1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f,  1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
	   -1.0f, -1.0f,  1.0f,  0.0f,  0.0f,  1.0f, 0.0f,  0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,

	   -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f,  0.0f,  0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,
	   -1.0f,  1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 1.0f,  1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,
	   -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f,  1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,
	   -1.0f, -1.0f, -1.0f, -1.0f,  0.0f,  0.0f, 0.0f,  1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,
	   -1.0f, -1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 0.0f,  0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,
	   -1.0f,  1.0f,  1.0f, -1.0f,  0.0f,  0.0f, 1.0f,  0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f,

		1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f,  0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f,
		1.0f,  1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 1.0f,  1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f,
		1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f,  1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f,
		1.0f, -1.0f, -1.0f,  1.0f,  0.0f,  0.0f, 0.0f,  1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f,
		1.0f, -1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 0.0f,  0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f,
		1.0f,  1.0f,  1.0f,  1.0f,  0.0f,  0.0f, 1.0f,  0.0f, 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f,

	   -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f,  1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
		1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 1.0f,  1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
		1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f,  0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
		1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 1.0f,  0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
	   -1.0f, -1.0f,  1.0f,  0.0f, -1.0f,  0.0f, 0.0f,  0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
	   -1.0f, -1.0f, -1.0f,  0.0f, -1.0f,  0.0f, 0.0f,  1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,

	   -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f,  1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f,
		1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 1.0f,  1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f,
		1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 1.0f,  0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f,
		1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 1.0f,  0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f,
	   -1.0f,  1.0f,  1.0f,  0.0f,  1.0f,  0.0f, 0.0f,  0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f,
	   -1.0f,  1.0f, -1.0f,  0.0f,  1.0f,  0.0f, 0.0f,  1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f
		};
		unsigned int indices[] = {
			0,1,2,3,4,5,
			6,7,8,9,10,11,
			12,13,14,15,16,17,
			18,19,20,21,22,23,
			24,25,26,27,28,29,
			30,31,32,33,34,35
		};
		unsigned int cubeVBO, cubeIBO, cubeVAO;
		glGenVertexArrays(1, &cubeVAO);
		glBindVertexArray(cubeVAO);

		glGenBuffers(1, &cubeVBO);
		glGenBuffers(1, &cubeIBO);
		glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, cubeIBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(6 * sizeof(float)));
		glEnableVertexAttribArray(3);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(8 * sizeof(float)));
		glEnableVertexAttribArray(4);
		glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(11 * sizeof(float)));
		Mesh* meshPtr = new Mesh(cubeVAO, cubeVBO, cubeIBO, 36);
		std::shared_ptr<Mesh> sharedPtr = std::shared_ptr<Mesh>(meshPtr);
		m_meshMap.insert({ "Cube", sharedPtr });
		return sharedPtr;

	}

	std::shared_ptr<Mesh> MeshManager::LoadPlane() noexcept {
		// Cada vertice tiene la siguiente forma
		// v = {p_x, p_y, p_z, n_x, n_y, n_z, uv_u, uv_v, t_x, t_y, t_z, b_x, b_y, b_z};
		float planeVertices[] = {
		-1.0f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
		1.0f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
		1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
		1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f, 1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
		-1.0f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
		-1.0f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f
		};

		unsigned int planeIndices[] =
		{
			0,1,2,3,4,5
		};

		unsigned int planeVBO, planeIBO, planeVAO;
		glGenVertexArrays(1, &planeVAO);
		glBindVertexArray(planeVAO);
		glGenBuffers(1, &planeVBO);
		glGenBuffers(1, &planeIBO);

		glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
		glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, planeIBO);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(planeIndices), planeIndices, GL_STATIC_DRAW);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(6 * sizeof(float)));
		glEnableVertexAttribArray(3);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(8 * sizeof(float)));
		glEnableVertexAttribArray(4);
		glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 14 * sizeof(float), (void*)(11 * sizeof(float)));
		Mesh* meshPtr = new Mesh(planeVAO, planeVBO, planeIBO, 6);
		std::shared_ptr<Mesh> sharedPtr = std::shared_ptr<Mesh>(meshPtr);
		m_meshMap.insert({ "Plane", sharedPtr });
		return sharedPtr;
	}
	void MeshManager::CleanUnusedMeshes() noexcept {
		for (auto i = m_meshMap.begin(), last = m_meshMap.end(); i != last;) {
			if (i->second.use_count() == 1) {
				i = m_meshMap.erase(i);
			}
			else {
				++i;
			}

		}
	}
	void MeshManager::ShutDown() noexcept {
		for (auto& entry : m_meshMap) {
			entry.second->ClearData();
		}


		m_meshMap.clear();
	}

	std::pair<std::shared_ptr<Mesh>, std::shared_ptr<Skeleton>> MeshManager::LoadMeshWithSkeleton(const std::filesystem::path& filePath,
		bool flipUvs) noexcept {

		const std::string& stringPath = filePath.string();
		Assimp::Importer importer;
		unsigned int postProcessFlags = flipUvs ? aiProcess_FlipUVs : 0;
		postProcessFlags |= aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_GenUVCoords | aiProcess_CalcTangentSpace;
		const aiScene* scene = importer.ReadFile(stringPath, postProcessFlags);


		/*
		if (!scene) {
			//En caso de fallar la carga se envia un mensaje de error y se carga un cubo.
			MONA_LOG_ERROR("MeshManager Error: Failed to open file with path {0}", stringPath);
			MONA_LOG_INFO("Loading default model");
			return LoadMesh(PrimitiveType::Cube);
		}
		*/
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
					MONA_ASSERT(face.mNumIndices == 3, "Model Manager Error: Can load meshes with faces that have {0} vertices", face.mNumIndices);
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

		std::vector<glm::mat4> invBindPoseMatrices;
		std::vector<std::string> jointNames;
		std::vector<std::int32_t> parentIndices;
		std::unordered_map<std::string, uint32_t> boneIndexMap;
		invBindPoseMatrices.reserve(boneInfo.size());
		jointNames.reserve(boneInfo.size());
		parentIndices.reserve(boneInfo.size());
		std::stack<int32_t> parentNodeIndices;
		sceneNodes.push(scene->mRootNode);

		parentNodeIndices.push(-1);
		
		while (!sceneNodes.empty())
		{
			const aiNode* currentNode = sceneNodes.top();
			sceneNodes.pop();
			int32_t parentIndex = parentNodeIndices.top();
			parentNodeIndices.pop();
			
			if (boneInfo.find(currentNode->mName.C_Str()) != boneInfo.end())
			{

				
				aiMatrix4x4& mat = boneInfo[currentNode->mName.C_Str()];
				glm::mat4 m = AssimpToGlmMatrix(mat);
				invBindPoseMatrices.push_back(m);
				jointNames.push_back(currentNode->mName.C_Str());
				parentIndices.push_back(parentIndex);
				boneIndexMap.insert(std::make_pair(currentNode->mName.C_Str(), static_cast<uint32_t>(parentIndices.size()-1)));
				int32_t newParentIndex = parentIndices.size() - 1;
				for (uint32_t j = 0; j < currentNode->mNumChildren; j++) {
					//Pusheamos los hijos y acumulamos la matrix de transformación
					sceneNodes.push(currentNode->mChildren[j]);
					parentNodeIndices.push(newParentIndex);
				}
			}

			else{
				for (uint32_t j = 0; j < currentNode->mNumChildren; j++) {
					//Pusheamos los hijos y acumulamos la matrix de transformación
					sceneNodes.push(currentNode->mChildren[j]);
					parentNodeIndices.push(parentIndex);
				}
			}

		}

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
					uint32_t index = boneIndexMap[std::string(bone->mName.C_Str())];
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
							break;

						}
					}

				}
				vertexOffset += meshOBJ->mNumVertices;
				
			}
			for (uint32_t j = 0; j < currentNode->mNumChildren; j++) {
				//Pusheamos los hijos y acumulamos la matrix de transformación
				sceneNodes.push(currentNode->mChildren[j]);
			}
		}



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
				MONA_LOG_INFO("!!!!!");

			}
		}
		/*
		for (uint32_t i = 0; i < parentIndices.size(); i++) {
			MONA_LOG_INFO("JointName: {0} Parent Index/Name: {1}/{2} ", jointNames[i], parentIndices[i],
				parentIndices[i] >= 0 ? jointNames[parentIndices[i]] : "NO PARENT");
		}*/
		//Comienza el paso de los datos en CPU a GPU usando OpenGL
		unsigned int modelVBO, modelIBO, modelVAO;
		glGenVertexArrays(1, &modelVAO);
		glBindVertexArray(modelVAO);

		glGenBuffers(1, &modelVBO);
		glGenBuffers(1, &modelIBO);
		glBindBuffer(GL_ARRAY_BUFFER, modelVBO);
		glBufferData(GL_ARRAY_BUFFER, static_cast<unsigned int>(vertices.size()) * sizeof(SkeletalMeshVertex), vertices.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, modelIBO);
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
		Mesh* meshPtr = new Mesh(modelVAO, modelVBO, modelIBO, static_cast<uint32_t>(faces.size()));
		Skeleton* skeletonPtr = new Skeleton(std::move(invBindPoseMatrices), std::move(jointNames), std::move(parentIndices));
		std::shared_ptr<Mesh> sharedPtr = std::shared_ptr<Mesh>(meshPtr);
		std::shared_ptr<Skeleton> skeletonSharedPtr = std::shared_ptr<Skeleton>(skeletonPtr);
		//Antes de retornar la malla recien cargada, insertamos esta al mapa para que cargas futuras sean mucho mas rapidas.
		//m_meshMap.insert({ stringPath, sharedPtr });
		return std::make_pair(sharedPtr, skeletonSharedPtr);

	}

	std::shared_ptr<AnimationClip> MeshManager::LoadAnimationClip(const std::filesystem::path& filePath, std::shared_ptr<Skeleton> skeleton) noexcept {

		const std::string& stringPath = filePath.string();
		Assimp::Importer importer;
		unsigned int postProcessFlags = aiProcess_Triangulate;
		const aiScene* scene = importer.ReadFile(stringPath, postProcessFlags);
		aiAnimation* animation = scene->mAnimations[0];
		float ticksPerSecond = animation->mTicksPerSecond != 0.0f? animation->mTicksPerSecond : 1.0f;
		float duration = animation->mDuration / animation->mTicksPerSecond;
		std::vector<AnimationClip::AnimationTrack> animationTracks;
		std::vector<std::string> trackNames;
		animationTracks.resize(animation->mNumChannels);
		trackNames.reserve(animation->mNumChannels);
		for (uint32_t i = 0; i < animation->mNumChannels; i++) {
			aiNodeAnim* track = animation->mChannels[i];
			trackNames.push_back(track->mNodeName.C_Str());
			AnimationClip::AnimationTrack& animationTrack = animationTracks[i];
			animationTrack.positions.reserve(track->mNumPositionKeys);
			animationTrack.positionTimeStamps.reserve(track->mNumPositionKeys);
			animationTrack.rotations.reserve(track->mNumRotationKeys);
			animationTrack.rotationTimeStamps.reserve(track->mNumRotationKeys);
			animationTrack.scales.reserve(track->mNumScalingKeys);
			animationTrack.scaleTimeStamps.reserve(track->mNumScalingKeys);
			for (uint32_t j = 0; j < track->mNumPositionKeys; j++) {
				animationTrack.positionTimeStamps.push_back(track->mPositionKeys[j].mTime / ticksPerSecond);
				animationTrack.positions.push_back(AssimpToGlmVec3(track->mPositionKeys[j].mValue));
			}
			for (uint32_t j = 0; j < track->mNumRotationKeys; j++) {
				animationTrack.rotationTimeStamps.push_back(track->mPositionKeys[j].mTime / ticksPerSecond);
				animationTrack.rotations.push_back(AssimpToGlmQuat(track->mRotationKeys[j].mValue));

			}
			for (uint32_t j = 0; j < track->mNumScalingKeys; j++) {
				animationTrack.scaleTimeStamps.push_back(track->mPositionKeys[j].mTime / ticksPerSecond);
				animationTrack.scales.push_back(AssimpToGlmVec3(track->mScalingKeys[j].mValue));

			}
		}
		AnimationClip* animationPtr = new AnimationClip(std::move(animationTracks),
			std::move(trackNames),
			skeleton,
			duration,
			ticksPerSecond);
		return std::shared_ptr<AnimationClip>(animationPtr);
	
	
	}
}