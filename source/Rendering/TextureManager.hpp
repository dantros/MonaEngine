#pragma once
#ifndef TEXTUREMANAGER_HPP
#define TEXTUREMANAGER_HPP
#include <memory>
#include <unordered_map>
#include <filesystem>
#include "Texture.hpp"
namespace Mona {
	class TextureManager {
	public:
		using TextureMap = std::unordered_map<std::string, std::shared_ptr<Texture>>;
		TextureManager(TextureManager const&) = delete;
		TextureManager& operator=(TextureManager const&) = delete;
		std::shared_ptr<Texture> LoadTexture(const std::filesystem::path& filePath,
			TextureMagnificationFilter magFilter = TextureMagnificationFilter::Linear,
			TextureMinificationFilter minFilter = TextureMinificationFilter::LinearMipmapLinear,
			WrapMode sWrapMode = WrapMode::Repeat,
			WrapMode tWrapMode = WrapMode::Repeat,
			bool genMipmaps = false) noexcept;
		void CleanUnusedTextures() noexcept;
		void ShutDown() noexcept;
		TextureManager() = default;
	private:
		TextureMap m_textureMap;
	};
}
#endif