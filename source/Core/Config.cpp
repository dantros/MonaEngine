#include "Config.hpp"
#include "Log.hpp"
#include "RootDirectory.hpp"
#include <fstream>
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

}