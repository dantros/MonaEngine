#include "Config.hpp"
#include "Log.hpp"
#include <fstream>
#include <whereami2cpp.h>

namespace Mona
{
	void Config::readFile(const std::string& path)
	{
		std::ifstream in(path);
		if(in.is_open())
		{
			std::string line;
			std::string::size_type lineNumber = 0;
			const std::string chars = "\t\n\v\f\r ";
			while (std::getline(in, line))
			{
				if (line.empty() || line[0] == '#' || line.find_first_not_of(chars) == std::string::npos)
					continue;
				auto delimeterPos = line.find_first_of("=");
				if (delimeterPos == 0 || delimeterPos == std::string::npos)
				{
					MONA_LOG_ERROR("Configuration: Incorrect line format (Line = {0}, Content = \"{1}\")", lineNumber, line);
					continue;
				}
				
				auto keyStart = line.find_first_not_of(chars);
				auto keyEnd = line.find_last_not_of(chars, delimeterPos-1);
				auto valueStart = line.find_first_not_of(chars, delimeterPos + 1);
				auto valueEnd = line.find_last_not_of(chars);
				
				if (keyStart == delimeterPos || (keyEnd - keyStart) < 0 || (valueEnd - valueStart) < 0)
				{
					MONA_LOG_ERROR("Configuration: Incorrect line format (Line = {0}, Content = \"{1}\")", lineNumber, line);
					continue;
				}
				m_configurations[line.substr(keyStart, keyEnd - keyStart + 1)] = line.substr(valueStart, valueEnd -  valueStart +1);
			}
			
		}
		else
		{
			MONA_LOG_ERROR("Configuration: Failed to open file {0}", path);
		}
		
		return;
	}

	void Config::loadDirectories()
	{
		std::string executablePathStr = whereami::get_executable_path();
		executablePath = executablePathStr;
		executableDir = executablePath.parent_path();

		configurationFile = executableDir;
		configurationFile.append("config.cfg");

		auto& config = Config::GetInstance();
		config.readFile(configurationFile.string());

		applicationAssetsDir = config.getValueOrDefault<std::string>("application_assets_dir", "Assets");
		engineAssetsDir = config.getValueOrDefault<std::string>("engine_assets_dir", "EngineAssets");

		loaded = true;
	}

	std::filesystem::path Config::SourcePath(const std::string &relativePath)
	{
		if (not loaded)
			loadDirectories();

		std::filesystem::path absolutePath = executableDir;
		absolutePath.append(relativePath);

		return absolutePath.string();
	}

	std::filesystem::path Config::ApplicationAssetPath(const std::string &relativePath)
	{
		if (not loaded)
			loadDirectories();

		std::filesystem::path absolutePath = applicationAssetsDir;
		absolutePath.append(relativePath);

		return absolutePath.string();
	}

	std::filesystem::path Config::EngineAssetPath(const std::string &relativePath)
	{
		if (not loaded)
			loadDirectories();

		std::filesystem::path absolutePath = engineAssetsDir;
		absolutePath.append(relativePath);

		return absolutePath.string();
	}

}