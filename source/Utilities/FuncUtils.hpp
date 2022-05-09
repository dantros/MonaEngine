#pragma once
#ifndef FUNCUTILS_HPP
#define FUNCUTILS_HPP

#include <vector>
#include <sstream>

namespace Mona {

	namespace funcUtils{
        template < typename T>
        inline int findIndex(const std::vector<T>& vecOfElements, const T& element)
        {
            int result;
            // Find given element in vector
            auto it = std::find(vecOfElements.begin(), vecOfElements.end(), element);
            if (it != vecOfElements.end())
            {
                result = std::distance(vecOfElements.begin(), it);
            }
            else
            {
                result = -1;
            }
            return result;
        }

        inline std::vector<std::string> splitString(const std::string& s, char delimiter)
        {
            std::vector<std::string> tokens;
            std::string token;
            std::istringstream tokenStream(s);
            while (std::getline(tokenStream, token, delimiter))
            {
                tokens.push_back(token);
            }
            return tokens;
        }

        template < typename T>
        inline std::string vec3vecToString(std::vector<T> vec) 
        {
            std::string result = "[ ";
            for (int i = 0; i < vec.size(); i++) {
                if (i < vec.size() - 1) {
                    result += "[" + std::to_string(vec[i][0]) + ", " + std::to_string(vec[i][1]) + ", " + std::to_string(vec[i][2]) + "]" + ", ";
                }
                else {
                    result += "[" + std::to_string(vec[i][0]) + ", " + std::to_string(vec[i][1]) + ", " + std::to_string(vec[i][2]) + "]";
                }
                
            }
            result += " ]";
            return result;
        }

        template < typename T>
        inline std::string vec4vecToString(std::vector<T> vec) 
        {
            std::string result = "[ ";
            for (int i = 0; i < vec.size(); i++) {
                if (i < vec.size() - 1) {
                    result += "[" + std::to_string(vec[i][0]) + ", " + std::to_string(vec[i][1]) + ", " + std::to_string(vec[i][2]) + ", " + std::to_string(vec[i][3]) + "]" + ", ";
                }
                else {
                    result += "[" + std::to_string(vec[i][0]) + ", " + std::to_string(vec[i][1]) + ", " + std::to_string(vec[i][2]) + ", " + std::to_string(vec[i][3]) + "]";
                }

            }
            result += " ]";
            return result;
        }
	}
    
}


#endif