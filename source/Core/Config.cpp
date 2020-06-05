#include "Config.hpp"
#include "Log.hpp"
#include "RootDirectory.hpp"
#include <fstream>
#include <iostream>
namespace Mona
{

	Config::Config() noexcept
	{
		m_configurations["windowWidth"] = "sdasdas1920";
		m_configurations["windowTitle"] = "FirstWindow";
	}


	void Config::readFile(const std::string& path)
	{
		//TODO(FILESYSTEM): This path calculation may be move from to another FilePathSystem or something like that
		const std::string new_path = source_directory + ("/" + path);
		std::ifstream in(new_path);
		std::string contents;
		in.seekg(0, std::ios::end);
		contents.resize(in.tellg());
		in.seekg(0, std::ios::beg);
		in.read(&contents[0], contents.size());
		in.close();
		std::cout << contents << "\n";
		return;
	}

}