#pragma once
#ifndef CONFIG_HPP
#define CONFIG_HPP
#include <unordered_map>
#include <string>
#include <sstream>
#include <filesystem>
#include <nlohmann/json.hpp>
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
			if (m_configurations.contains(key))
				return m_configurations[key];

			return defaultValue;
		}

		std::filesystem::path getPathRelativeToExecutable(const std::string &relativePath);
		std::filesystem::path getPathOfApplicationAsset(const std::string &relativePath);
		std::filesystem::path getPathOfEngineAsset(const std::string &relativePath);

	private:
		Config() noexcept {}
		nlohmann::json m_configurations;

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
}





#endif