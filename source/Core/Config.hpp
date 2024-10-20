#pragma once
#ifndef CONFIG_HPP
#define CONFIG_HPP
#include <unordered_map>
#include <string>
#include <sstream>
#include <filesystem>
#include "Log.hpp"

namespace Mona {
	class Config {
	public:
		Config(Config const&) = delete;
		Config& operator=(Config const&) = delete;
		
		static Config& GetInstance()
		{
			static Config instance;
			return instance;
		}

		void loadDefault();

		template <typename T>
		inline T getValueOrDefault(const std::string& key, const T& defaultValue) const noexcept
		{
			auto it = m_configurations.find(key);
			if (it != m_configurations.end())
			{
				std::istringstream istr(it->second);
				T returnValue;
				if (!(istr >> returnValue))
				{
					MONA_LOG_ERROR("Configuration: Failed to transform {0}'s value from {1} into an {2}", it->first, it->second, typeid(T).name());
					return defaultValue;
				}
				return returnValue;
			}
			return defaultValue;
		}

		std::filesystem::path getPathRelativeToExecutable(const std::string &relativePath);
		std::filesystem::path getPathOfApplicationAsset(const std::string &relativePath);
		std::filesystem::path getPathOfEngineAsset(const std::string &relativePath);

	private:
		Config() noexcept {}
		std::unordered_map<std::string, std::string> m_configurations;

		bool m_loaded = false;
		void loadDirectories();
		void readFile(const std::string& path);

		/* these values are set with the executable path, cannot be changed. */
		std::filesystem::path m_executablePath;
		std::filesystem::path m_executableDir;

		/* configuration file should be next to the executable. */
		std::filesystem::path m_configurationFile;

		/* extratced from the configuration file. */
		std::filesystem::path m_applicationAssetsDir;
		std::filesystem::path m_engineAssetsDir;
	};

	template <>
	inline std::string Config::getValueOrDefault(const std::string& key, const std::string& defaultValue) const noexcept
	{
		auto it = m_configurations.find(key);
		if (it != m_configurations.end()) {
			return it->second;
		}
		return defaultValue;
	}
	
}





#endif