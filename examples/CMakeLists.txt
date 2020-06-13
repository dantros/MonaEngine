add_executable(Sandbox main.cpp)
set_property(TARGET Sandbox PROPERTY CXX_STANDARD 20)
#Cambiar en el futuro, Ejemplos no deberian necesitar las otras librerias solo los includes.
target_link_libraries(Sandbox PRIVATE MonaEngine ${THIRD_PARTY_LIBRARIES})
target_include_directories(Sandbox PRIVATE ${MONA_INCLUDE_DIRECTORY} ${THIRD_PARTY_INCLUDE_DIRECTORIES})