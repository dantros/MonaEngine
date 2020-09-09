#pragma once
#ifndef MODELMANAGER_HPP
#define MODELMANAGER_HPP
#include <cstdint>
namespace Mona {
	class ModelManager {
	public:
		struct ModelHandle {
		public:
			ModelHandle(uint32_t id, uint32_t c) : ID(id), count(c) {}
			uint32_t ID;
			uint32_t count;
		};
		enum class PrimitiveType {
			Plane,
			Cube,
			Sphere,
			PrimitiveCount
		};
		ModelManager(ModelManager const&) = delete;
		ModelManager& operator=(ModelManager const&) = delete;
		ModelHandle LoadModel(PrimitiveType type) const noexcept;
		void StartUp() noexcept;
		static ModelManager& GetInstance() {
			static ModelManager instance;
			return instance;
		}
	private:
		void LoadSphere() noexcept;
		ModelHandle m_planeModel = ModelHandle(0,0);
		ModelHandle m_cubeModel = ModelHandle(0,0);
		ModelHandle m_sphereModel = ModelHandle(0,0);
		ModelManager() {};
	};
}
#endif