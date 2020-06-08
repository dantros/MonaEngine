#pragma once
#ifndef CONFIG_HPP
#define CONFIG_HPP
#include <unordered_map>
#include <string>
#include <sstream>
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
		void readFile(const std::string& path); 
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



	private:
		Config() noexcept {}
		std::unordered_map<std::string, std::string> m_configurations;
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