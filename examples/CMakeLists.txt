function(Add_Example TARGETNAME FILENAME)
	add_executable(${TARGETNAME} ${FILENAME})
	set_property(TARGET ${TARGETNAME} PROPERTY CXX_STANDARD 20)
	set_property(TARGET ${TARGETNAME} PROPERTY FOLDER Examples)
	target_link_libraries(${TARGETNAME} PRIVATE MonaEngine)
	target_include_directories(${TARGETNAME} PRIVATE ${MONA_INCLUDE_DIRECTORY} ${THIRD_PARTY_INCLUDE_DIRECTORIES})

endfunction(Add_Example)
#add_executable(Sandbox main.cpp)
#set_property(TARGET Sandbox PROPERTY CXX_STANDARD 20)
#Cambiar en el futuro, Ejemplos no deberian necesitar las otras librerias solo los includes.  ${THIRD_PARTY_LIBRARIES}
#target_link_libraries(Sandbox PRIVATE MonaEngine)
#target_include_directories(Sandbox PRIVATE ${MONA_INCLUDE_DIRECTORY} ${THIRD_PARTY_INCLUDE_DIRECTORIES})
Add_Example(Sanbox main.cpp)
Add_Example(Example0_BasicPrimitives_NoTransform Example0_BasicPrimitives_NoTransform.cpp)
Add_Example(Example1_BasicPrimitives Example1_BasicPrimitives.cpp)
Add_Example(Example2_BasicPrimitives_Camera Example2_BasicPrimitives_Camera.cpp)
Add_Example(Example3_LoadModel Example3_LoadModel.cpp)

