function(Add_Test TARGETNAME FILENAME)
	add_executable(${TARGETNAME} ${FILENAME})
	set_property(TARGET ${TARGETNAME} PROPERTY CXX_STANDARD 20)
	set_property(TARGET ${TARGETNAME} PROPERTY FOLDER Tests)
	target_link_libraries(${TARGETNAME} PRIVATE MonaEngine)
	target_include_directories(${TARGETNAME} PRIVATE ${MONA_INCLUDE_DIRECTORY} ${THIRD_PARTY_INCLUDE_DIRECTORIES})

endfunction(Add_Test)

Add_Test(Test000_ComponentManager Test000_ComponentManager.cpp)
Add_Test(Test001_GameObjectManager Test001_GameObjectManager.cpp)



