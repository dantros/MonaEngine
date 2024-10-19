#include "RootDirectory.hpp"
#include <whereami2cpp.h>
#include <filesystem>

namespace Mona{
	std::filesystem::path SourceDirectoryData::SourcePath(const std::string &relativePath){
		std::string executablePathStr = whereami::get_executable_path();
		std::filesystem::path executablePath(executablePathStr);
		std::filesystem::path executableDir = executablePath.parent_path();
		std::filesystem::path absolutePath = executableDir;
		absolutePath.append(relativePath);

		return absolutePath.string();
	}
	
}
