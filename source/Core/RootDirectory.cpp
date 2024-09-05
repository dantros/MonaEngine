/*
	This field was automatically created with CMake please don't modify it
*/
#include "RootDirectory.hpp"
#include <filesystem>
namespace Mona{
	std::string source_directory = "D:/Universidad/2022-1/Trabajo_de_Titulo/codigo/MonaEngine_IK/";
	void SourceDirectoryData::SetSourceDirectory(std::string newSourceDirectory){
		if(newSourceDirectory.back() != '/' && newSourceDirectory.back() != '\\') {
			newSourceDirectory.append("/");
		}
		source_directory = newSourceDirectory;
	}
	std::filesystem::path SourceDirectoryData::SourcePath(const std::string &relative_path){
		return source_directory + relative_path;
	}
	
}
