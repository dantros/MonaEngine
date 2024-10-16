#include "Mesh.hpp"

#include "../Core/Log.hpp"
#include "../Core/AssimpTransformations.hpp"
#include <glm/glm.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <vector>
#include <stack>
#include <glad/glad.h>
namespace Mona {

	struct MeshVertex {
		glm::vec3 position;
		glm::vec3 normal;
		glm::vec2 uv;
		glm::vec3 tangent;
		glm::vec3 bitangent;
	};

	Mesh::~Mesh() {
		if (m_vertexArrayID)
			ClearData();
	}
	void Mesh::ClearData() noexcept {
		MONA_ASSERT(m_vertexArrayID, "Mesh Error: Trying to delete already deleted mesh");
		MONA_ASSERT(m_vertexBufferID, "Mesh Error: Trying to delete already deleted mesh");
		MONA_ASSERT(m_indexBufferID, "Mesh Error: Trying to delete already deleted mesh");
		glDeleteBuffers(1, &m_vertexBufferID);
		glDeleteBuffers(1, &m_indexBufferID);
		glDeleteVertexArrays(1, &m_vertexArrayID);
		m_vertexArrayID = 0;
	}

	Mesh::Mesh(const std::string& filePath, bool flipUVs) :
		m_vertexArrayID(0),
		m_vertexBufferID(0),
		m_indexBufferID(0),
		m_indexBufferCount(0)
	{
		Assimp::Importer importer;
		unsigned int postProcessFlags = flipUVs ? aiProcess_FlipUVs : 0;
		postProcessFlags |= aiProcess_Triangulate | aiProcess_GenNormals | aiProcess_GenUVCoords | aiProcess_CalcTangentSpace;
		const aiScene* scene = importer.ReadFile(filePath, postProcessFlags);



		if (!scene) {
			//En caso de fallar la carga se envia un mensaje de error.
			MONA_LOG_ERROR("Mesh Error: Failed to open file with path {0}", filePath);
			return;
		}

		std::vector<MeshVertex> vertices;
		std::vector<unsigned int> faces;
		size_t numVertices = 0;
		size_t numFaces = 0;
		//El primer paso consiste en contar el numero de vertices y caras totales
		//de esta manera se puede reservar memoria inmediatamente evitando realocaci�n de memoria
		for (uint32_t i = 0; i < scene->mNumMeshes; i++) {
			numVertices += scene->mMeshes[i]->mNumVertices;
			numFaces += scene->mMeshes[i]->mNumFaces;
		}

		vertices.reserve(numVertices);
		faces.reserve(numFaces);

		//El grafo de la escena se reccorre usando DFS (Depth Search First) usando dos stacks.
		std::stack<const aiNode*> sceneNodes;
		std::stack<aiMatrix4x4> sceneTransforms;
		//Luego pusheamos informaci�n asociada a la raiz del grafo
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
					MONA_ASSERT(face.mNumIndices == 3, "Mesh Error: Can load meshes with faces that have {0} vertices", face.mNumIndices);
					faces.push_back(face.mIndices[0] + offset);
					faces.push_back(face.mIndices[1] + offset);
					faces.push_back(face.mIndices[2] + offset);
				}
				offset += meshOBJ->mNumVertices;


			}

			for (uint32_t j = 0; j < currentNode->mNumChildren; j++) {
				//Pusheamos los hijos y acumulamos la matrix de transformaci�n
				sceneNodes.push(currentNode->mChildren[j]);
				sceneTransforms.push(currentNode->mChildren[j]->mTransformation * currentTransform);
			}
		}

		//Comienza el paso de los datos en CPU a GPU usando OpenGL
		m_indexBufferCount = static_cast<uint32_t>(faces.size());
		glGenVertexArrays(1, &m_vertexArrayID);
		glBindVertexArray(m_vertexArrayID);

		glGenBuffers(1, &m_vertexBufferID);
		glGenBuffers(1, &m_indexBufferID);
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferID);
		glBufferData(GL_ARRAY_BUFFER, static_cast<unsigned int>(vertices.size()) * sizeof(MeshVertex), vertices.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBufferID);
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

	}

	Mesh::Mesh(PrimitiveType type) :
		m_vertexArrayID(0),
		m_vertexBufferID(0),
		m_indexBufferID(0),
		m_indexBufferCount(0)
	{
		switch (type)
		{
		case Mona::Mesh::PrimitiveType::Plane:
		{
			CreatePlane();
			break;
		}
		case Mona::Mesh::PrimitiveType::Cube:
		{
			CreateCube();
			break;
		}
		case Mona::Mesh::PrimitiveType::Sphere:
		{
			CreateSphere();
			break;
		}
		default:
		{

			CreateSphere();
			break;
		}
		}
	}

	glm::vec3 surfaceColor(glm::vec3 baseColor, float modifier) {
		return { baseColor[0] * modifier, baseColor[1] * modifier / 2, baseColor[2] };
	}


	int orientationTest(const glm::vec3& v1, const glm::vec3& v2, const glm::vec3& testV, float epsilon) { //arista de v1 a v2, +1 si el punto esta a arriba, -1 abajo,0 si es colineal, error(-2) si v1 y v2 iguales
		float x1 = v1[0];
		float y1 = v1[1];
		float x2 = v2[0];
		float y2 = v2[1];

		int orientationV1V2 = 1;
		if (x1 > x2) {
			orientationV1V2 = -1;
		}

		if (std::abs(testV[0] - v1[0]) <= epsilon && std::abs(testV[1] - v1[1]) <= epsilon ||
			std::abs(testV[0] - v2[0]) <= epsilon && std::abs(testV[1] - v2[1]) <= epsilon) {
			return 0;
		}

		if (x1 == x2 and y1 == y2) {
			MONA_LOG_ERROR("vertices are equal!");
			return -2;
		}
		if (x1 == x2) {
			if (y1 > y2) { orientationV1V2 = -1; }
			if (testV[0] == x1) { return 0; }
			else if (testV[0] > x1) { return -1 * orientationV1V2; }
			else { return 1 * orientationV1V2; }
		}


		float yOnLine = ((y2 - y1) / (x2 - x1)) * (testV[0] - x1) + y1;
		if (testV[1] > yOnLine) { return 1 * orientationV1V2; }
		else if (testV[1] < yOnLine) { return -1 * orientationV1V2; }
		else { return 0; }
	}

	Mesh::Mesh(const glm::vec2& minXY, const glm::vec2& maxXY, int numInnerVerticesWidth, int numInnerVerticesHeight,
		float (*heightFunc)(float, float)) :
		m_vertexArrayID(0),
		m_vertexBufferID(0),
		m_indexBufferID(0),
		m_indexBufferCount(0)
	{
		//Un vertice de la malla se ve como
		// v = {pos_x, pos_y, pos_z, normal_x, normal_y, normal_z, uv_u, uv_v, tangent_x, tangent_y, tangent_z}
		std::vector<float> vertices;
		std::vector<unsigned int> faces;
		size_t numVertices = 0;
		size_t numFaces = 0;

		float stepX = (maxXY[0] - minXY[0]) / (numInnerVerticesWidth + 1);
		float stepY = (maxXY[1] - minXY[1]) / (numInnerVerticesHeight + 1);
		for (int i = 0; i < numInnerVerticesWidth + 2; i++) {
			float x = minXY[0] + stepX * i;
			for (int j = 0; j < numInnerVerticesHeight + 2; j++) {
				float y = minXY[1] + stepY * j;
				float z = heightFunc(x, y);
				numVertices += 1;
				vertices.insert(vertices.end(), { x, y, z, 0, 0, 0, 0, 0, 0, 0, 0 }); // falta rellenar valores
			}
		}

		// indexing func
		auto index = [&](int i, int j)
		{
			return i * (numInnerVerticesHeight + 2) + j;
		};

		// We generate quads for each cell connecting 4 neighbor vertices
		for (int i = 0; i < numInnerVerticesWidth + 1; i++) {
			for (int j = 0; j < numInnerVerticesHeight + 1; j++) {
				// Getting indices for all vertices in this quad
				unsigned int isw = index(i, j);
				unsigned int ise = index(i + 1, j);
				unsigned int ine = index(i + 1, j + 1);
				unsigned int inw = index(i, j + 1);

				numFaces += 2;
				faces.insert(faces.end(), { isw, ise, ine, ine, inw, isw }); // falta rellenar las normales
			}
		}
		std::vector<glm::vec3> vertexPositions;
		std::vector<glm::vec3> groupedFaces;
		std::unordered_map<int, std::vector<glm::vec3>> oneFacePerVertex;
		vertexPositions.reserve(numVertices);
		oneFacePerVertex.reserve(numVertices);
		groupedFaces.reserve(faces.size());
		for (int i = 0; i < vertices.size(); i += 11) {
			glm::vec3 v(vertices[i], vertices[i + 1], vertices[i + 2]);
			vertexPositions.push_back(v);
		}
		for (int i = 0; i < faces.size(); i += 3) {
			glm::vec3 v1 = vertexPositions[faces[i]];
			glm::vec3 v2 = vertexPositions[faces[i + 1]];
			glm::vec3 v3 = vertexPositions[faces[i + 2]];
			glm::vec3 f = { faces[i], faces[i + 1], faces[i + 2] };
			int orientation = orientationTest(v1, v2, v3, 0.00001f);
			if (orientation == -1) {
				f = { faces[i + 1], faces[i], faces[i + 2] };
			}
			groupedFaces.push_back(f);
			// Guardamos una cara por cada vertice para extraer normales y tangentes
			if (oneFacePerVertex[faces[i]].empty()) {
				oneFacePerVertex[faces[i]].push_back(vertexPositions[faces[i]]);
				oneFacePerVertex[faces[i]].push_back(vertexPositions[faces[i + 1]]);
				oneFacePerVertex[faces[i]].push_back(vertexPositions[faces[i + 2]]);
			}
			if (oneFacePerVertex[faces[i + 1]].empty()) {
				oneFacePerVertex[faces[i + 1]].push_back(vertexPositions[faces[i]]);
				oneFacePerVertex[faces[i + 1]].push_back(vertexPositions[faces[i + 1]]);
				oneFacePerVertex[faces[i + 1]].push_back(vertexPositions[faces[i + 2]]);
			}
			if (oneFacePerVertex[faces[i + 2]].empty()) {
				oneFacePerVertex[faces[i + 2]].push_back(vertexPositions[faces[i]]);
				oneFacePerVertex[faces[i + 2]].push_back(vertexPositions[faces[i + 1]]);
				oneFacePerVertex[faces[i + 2]].push_back(vertexPositions[faces[i + 2]]);
			}
		}

		// Rellenar valores faltantes en vertices
		for (int i = 0; i < vertices.size(); i += 11) {
			int vertexIndex = i / 11;
			glm::vec3 v1 = oneFacePerVertex[vertexIndex][0];
			glm::vec3 v2 = oneFacePerVertex[vertexIndex][1];
			glm::vec3 v3 = oneFacePerVertex[vertexIndex][2];
			glm::vec3 normal = glm::normalize(glm::cross((v2 - v1), (v3 - v1)));
			vertices[i + 3] = normal[0];
			vertices[i + 4] = normal[1];
			vertices[i + 5] = normal[2];
			// uv
			vertices[i + 6] = 0.0f;
			vertices[i + 7] = 0.0f;
			glm::vec3 tangent = glm::normalize(v2 - v1);
			vertices[i + 8] = tangent[0];
			vertices[i + 9] = tangent[1];
			vertices[i + 10] = tangent[2];
		}

		m_heightMap = HeightMap({ minXY[0], minXY[1] }, { maxXY[0], maxXY[1] }, heightFunc);

		//Comienza el paso de los datos en CPU a GPU usando OpenGL
		m_indexBufferCount = static_cast<uint32_t>(faces.size());
		glGenVertexArrays(1, &m_vertexArrayID);
		glBindVertexArray(m_vertexArrayID);

		glGenBuffers(1, &m_vertexBufferID);
		glGenBuffers(1, &m_indexBufferID);
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferID);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * static_cast<unsigned int>(vertices.size()), vertices.data(), GL_STATIC_DRAW);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_indexBufferID);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * static_cast<unsigned int>(faces.size()), faces.data(), GL_STATIC_DRAW);
		//Un vertice de la malla se ve como
		// v = {pos_x, pos_y, pos_z, normal_x, normal_y, normal_z, uv_u, uv_v, tangent_x, tangent_y, tangent_z}
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 11 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 11 * sizeof(float), (void*)(3 * sizeof(float)));
		glEnableVertexAttribArray(2);
		glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 11 * sizeof(float), (void*)(6 * sizeof(float)));
		glEnableVertexAttribArray(3);
		glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 11 * sizeof(float), (void*)(8 * sizeof(float)));
		glEnableVertexAttribArray(4);
		glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, 11 * sizeof(float), (void*)(11 * sizeof(float)));

	}

	void Mesh::CreateCube() noexcept {
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
		m_vertexArrayID = cubeVAO;
		m_vertexBufferID = cubeVBO;
		m_indexBufferID = cubeIBO;
		m_indexBufferCount = 36;
	}

	void Mesh::CreatePlane() noexcept {
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
		m_vertexArrayID = planeVAO;
		m_vertexBufferID = planeVBO;
		m_indexBufferID = planeIBO;
		m_indexBufferCount = 6;
	}

	void Mesh::CreateSphere() noexcept {
		//Esta implementaci�n de la creacion procedural de la malla de una esfera
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
				float u = (float)j / (float)sectorCount;
				float v = (float)i / (float)stackCount;
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
		glBufferData(GL_ARRAY_BUFFER, static_cast<unsigned int>(vertices.size()) * sizeof(float), vertices.data(), GL_STATIC_DRAW);
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
		m_vertexArrayID = sphereVAO;
		m_vertexBufferID = sphereVBO;
		m_indexBufferID = sphereIBO;
		m_indexBufferCount = static_cast<uint32_t>(indices.size());
	}

}