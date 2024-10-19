#pragma once
#ifndef ROOTDIRECTORY_HPP
#define ROOTDIRECTORY_HPP

#include <filesystem>

namespace Mona{
	class SourceDirectoryData{
		public:
		static std::filesystem::path SourcePath(const std::string &relativePath);
	};
	
}

#endif
