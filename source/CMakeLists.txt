set(MONA_HEADERS 
				Core/Common.hpp 
				Core/Log.hpp
				Core/Config.hpp
				Core/RootDirectory.hpp)
set(MONA_SOURCES 
				Core/Log.cpp
				Core/Config.cpp)

add_executable(Sandbox main.cpp ${MONA_HEADERS} ${MONA_SOURCES})
target_include_directories(Sandbox PRIVATE ${THIRD_PARTY_INCLUDE_DIRECTORIES})
target_link_libraries(Sandbox PRIVATE ${THIRD_PARTY_LIBRARIES})
set_property(TARGET Sandbox PROPERTY CXX_STANDARD 20)


