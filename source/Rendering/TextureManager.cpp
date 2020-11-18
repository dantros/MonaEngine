#include "TextureManager.hpp"
namespace Mona {
	std::shared_ptr<Texture> TextureManager::LoadTexture(const std::filesystem::path& filePath,
		TextureMagnificationFilter magFilter,
		TextureMinificationFilter minFilter,
		WrapMode sWrapMode,
		WrapMode tWrapMode,
		bool genMipmaps) noexcept
	{
		const std::string stringPath = filePath.string();
		auto it = m_textureMap.find(stringPath);
		//Solo pasar a crear la textura si no existe una entrada en el mapa con el mismo path
		if (it != m_textureMap.end())
			return it->second;
		Texture* texturePtr = new Texture(stringPath, magFilter, minFilter, sWrapMode, tWrapMode, genMipmaps);
		std::shared_ptr<Texture> textureSharedPtr = std::shared_ptr<Texture>(texturePtr);
		//Antes de retornar la texture se inserta una entrada al mapa 
		m_textureMap.insert({ stringPath, textureSharedPtr });
		return textureSharedPtr;
	}

	void TextureManager::CleanUnusedTextures() noexcept
	{
		for (auto i = m_textureMap.begin(), last = m_textureMap.end(); i != last;) {
			if (i->second.use_count() == 1) {
				i = m_textureMap.erase(i);
			}
			else {
				++i;
			}

		}
	}

	void TextureManager::ShutDown() noexcept
	{
		for (auto& entry : m_textureMap) {
			(entry.second)->ClearData();
		}
		m_textureMap.clear();
	}


}