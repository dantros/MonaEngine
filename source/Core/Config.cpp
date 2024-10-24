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
		m_configurations["OpenGL_major_version"] = 4;
		m_configurations["OpenGL_minor_version"] = 5;

		// Audio Setting
		m_configurations["N_OPENAL_SOURCES"] = 32;

		// Game Object Settings
		m_configurations["expected_number_of_gameobjects"] = 1200;
	}

	void Config::readFile(const std::string& path)
	{
		std::ifstream file(path);
		file >> m_configurations;
	}

	void Config::loadDirectories()
	{
		// TODO: change config file format to json

		std::string executablePathStr = whereami::get_executable_path();
		m_executablePath = executablePathStr;
		m_executableDir = m_executablePath.parent_path();

		m_configurationFile = m_executableDir;
		m_configurationFile.append("config.json");

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
			MONA_LOG_INFO("There is no configuration file \"config.json\" next to the executable. Using defaults.");

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