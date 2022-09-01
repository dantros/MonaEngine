#pragma once
#ifndef FUNCUTILS_HPP
#define FUNCUTILS_HPP

#include <vector>
#include <sstream>
#include "Log.hpp"
#include <iostream>

namespace Mona {

	namespace funcUtils{
        template < typename T>
        inline int findIndex(const std::vector<T>& vecOfElements, const T& element)
        {
            int result;
            // Find given element in vector
            auto it = std::find(vecOfElements.begin(), vecOfElements.end(), element);
            if (it != vecOfElements.end()) { result = std::distance(vecOfElements.begin(), it); }
            else { result = -1; }
            return result;
        }

        template < typename T>
        inline void sortUnique(std::vector<T>& vecOfElements, bool sortDescending = false)
        {
            if (vecOfElements.size() == 0) { return; }
            if (sortDescending) { std::sort(vecOfElements.begin(), vecOfElements.end(), std::greater<T>()); }
            else { std::sort(vecOfElements.begin(), vecOfElements.end()); }
            std::vector<T> temp = {};
            temp.push_back(vecOfElements[0]);
            for (int i = 1; i < vecOfElements.size(); i++) {
                if (vecOfElements[i] != vecOfElements[i-1]) {
                    temp.push_back(vecOfElements[i]);
                }
            }
            vecOfElements = temp;
        }

        template < typename T>
        inline void removeDuplicates(std::vector<T>& vecOfElements)
        {

            std::vector<T> uniqueElements = vecOfElements;
            sortUnique(uniqueElements);
            for (int i = 0; i < uniqueElements.size(); i++) {
                while (1 < std::count(vecOfElements.begin(), vecOfElements.end(), uniqueElements[i])) {
                    vecOfElements.erase(vecOfElements.begin() + findIndex(vecOfElements, uniqueElements[i]));
                }
            }
        }

        template < typename T>
        inline int findIndexSubArray(const std::vector<T>& vecOfElements, const T& element, int indexStart, int indexEnd)
        {
            if (indexStart < 0 || indexStart >= vecOfElements.size() || indexEnd < 0 || indexEnd >= vecOfElements.size()) {
                MONA_LOG_ERROR("Index out of bounds");
                return -1;
            };
            if (indexEnd < indexStart) { return -1; }
            int result;
            // Find given element in vector
            auto start = vecOfElements.begin() + indexStart;
            auto end = vecOfElements.begin() + indexEnd + 1;
            auto it = std::find(start, end, element);
            if (it != end) { result = std::distance(vecOfElements.begin(), it); }
            else { result = -1; }
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
        inline std::string vecToString(std::vector<T> vec)
        {
            std::string result = "[ ";
            for (int i = 0; i < vec.size(); i++) {
                if (i < vec.size() - 1) {
                    result += std::to_string(vec[i]) + ", ";
                }
                else {
                    result += std::to_string(vec[i]);
                }
            }
            result += " ]";
            return result;
        }

        template < typename T>
        inline std::vector<int> minValueIndex_multiple(std::vector<T> vec) {
            T minVal = std::numeric_limits<T>::max();
            for (int i = 0; i < vec.size(); i++) {
                if (vec[i] < minVal) {
                    minVal = vec[i];
                }
            }
            std::vector<int> minIndexes;
            for (int i = 0; i < vec.size(); i++) {
                if (vec[i] == minVal) {
                    minIndexes.push_back(i);
                }
            }
            return minIndexes;
        }

        inline bool conditionVector_OR(std::vector<bool> condVec) {
            for (int i = 0; i < condVec.size(); i++) {
                if (condVec[i]) {
                    return true;
                }
            }
            return false;
        }

        inline bool conditionVector_AND(std::vector<bool> condVec) {
            for (int i = 0; i < condVec.size(); i++) {
                if (!condVec[i]) {
                    return false;
                }
            }
            return true;
        }

        template <typename T>
        inline T stdVectorSum(std::vector<T> vec) {
            T result = 0;
            for (int i = 0; i < vec.size(); i++) {
                result += vec[i];
            }
            return result;
        }

        template < typename T>
        inline T lerp(T minVal, T maxVal, float fraction) {
            return minVal + (maxVal - minVal) * fraction;
        }


        inline float getFraction(float minVal, float maxVal, float val) {
            MONA_ASSERT(minVal != maxVal, "FuncUtils: extreme values cannot be equal.");
            return (val - minVal) / (maxVal - minVal);
        }


		inline void epsilonAdjustment_add(float& value, float epsilon) {
            MONA_ASSERT(0 < epsilon, "FuncUtils: epsilon must be greater than 0.");
            if (epsilon == 0) { return; }
			while (value + epsilon == value) {
				epsilon *= 2;
			}
			value += epsilon;
		}

		inline void epsilonAdjustment_subtract(float& value, float epsilon) {
            MONA_ASSERT(0 < epsilon, "FuncUtils: epsilon must be greater than 0.");
            if (epsilon == 0) { return; }
			while (value - epsilon == value) {
				epsilon *= 2;
			}
			value -= epsilon;
		}

        template < typename T>
        inline int getSign(T val) {
            if (val == 0) {
                return 1;
            }
            float sign = val / (float)abs(val);
            if (sign < 0) {
                return -1;
            }
            return  1;
        }
	}
    
}


#endif