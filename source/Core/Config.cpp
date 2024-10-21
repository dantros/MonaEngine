#include "Config.hpp"
#include "Log.hpp"
#include <fstream>
#include <whereami2cpp.h>

namespace Mona
{
	void Config::loadDefault()
	{
		// Window Settings
		m_configurations["windowTitle"] = "MonaEngine Application";

		// OpenGL Settings
		m_configurations["OpenGL_major_version"] = "4";
		m_configurations["OpenGL_minor_version"] = "5";

		// Audio Setting
		m_configurations["N_OPENAL_SOURCES"] = "32";

		// Game Object Settings
		m_configurations["expected_number_of_gameobjects"] = "1200";
	}

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
		// TODO: change config file format to json

		std::string executablePathStr = whereami::get_executable_path();
		m_executablePath = executablePathStr;
		m_executableDir = m_executablePath.parent_path();

		m_configurationFile = m_executableDir;
		m_configurationFile.append("config.cfg");

		/* The configuration file allow us to specify asset folders for the application and for the engine.
		   Those paths must be absolute, this is meant to help development only.
		 */
		if (std::filesystem::is_regular_file(m_configurationFile))
		{
			readFile(m_configurationFile.string());

			std::string applicationAssetsDirStr = getValueOrDefault<std::string>("application_assets_dir", "X");
			if (applicationAssetsDirStr == "X")
			{
				/* If we do not have a configuration file, we look for the asset folders next to the executable. */
				m_applicationAssetsDir = m_executableDir;
				m_applicationAssetsDir.append("Assets");
			}
			else
			{
				m_applicationAssetsDir = applicationAssetsDirStr;
			}

			std::string engineAssetsDirStr = getValueOrDefault<std::string>("engine_assets_dir", "X");
			if (engineAssetsDirStr == "X")
			{
				/* If we do not have a configuration file, we look for the asset folders next to the executable. */
				m_engineAssetsDir = m_executableDir;
				m_engineAssetsDir.append("EngineAssets");
			}
			else
			{
				m_engineAssetsDir = engineAssetsDirStr;
			}
		}
		else
		{
			MONA_LOG_INFO("There is no configuration file \"config.cfg\" next to the executable. Using defaults.");

			loadDefault();

			m_configurationFile = "";

			m_applicationAssetsDir = m_executableDir;
			m_applicationAssetsDir.append("Assets");

			m_engineAssetsDir = m_executableDir;
			m_engineAssetsDir.append("EngineAssets");
		}

		MONA_ASSERT(std::filesystem::is_directory(m_applicationAssetsDir), "Invalid directory for application assets: " + m_applicationAssetsDir.string() + ".");
		MONA_ASSERT(std::filesystem::is_directory(m_engineAssetsDir), "Invalid directory for engine assets: " + m_engineAssetsDir.string() + ".");

		m_loaded = true;
	}

	std::filesystem::path Config::getPathRelativeToExecutable(const std::string &relativePath)
	{
		if (not m_loaded)
			loadDirectories();

		std::filesystem::path absolutePath = m_executableDir;
		absolutePath.append(relativePath);

		return absolutePath.string();
	}

	std::filesystem::path Config::getPathOfApplicationAsset(const std::string &relativePath)
	{
		if (not m_loaded)
			loadDirectories();

		std::filesystem::path absolutePath = m_applicationAssetsDir;
		absolutePath.append(relativePath);

		return absolutePath.string();
	}

	std::filesystem::path Config::getPathOfEngineAsset(const std::string &relativePath)
	{
		if (not m_loaded)
			loadDirectories();

		std::filesystem::path absolutePath = m_engineAssetsDir;
		absolutePath.append(relativePath);

		return absolutePath.string();
	}

}