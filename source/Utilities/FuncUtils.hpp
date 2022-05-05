#pragma once
#ifndef FUNCUTILS_HPP
#define FUNCUTILS_HPP
#include <vector>

namespace Mona {

	namespace funcUtils{
        template < typename T>
        int findIndex(const std::vector<T>& vecOfElements, const T& element)
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

        std::vector<std::string> splitString(const std::string& s, char delimiter)
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
		
	}
    
}











#endif