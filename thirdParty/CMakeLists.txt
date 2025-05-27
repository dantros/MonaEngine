set_property(GLOBAL PROPERTY USE_FOLDERS TRUE)
define_property(
    TARGET
    PROPERTY FOLDER
    INHERITED
    BRIEF_DOCS "Set the folder name."
    FULL_DOCS  "Use to organize targets in an IDE."
)

function(add_subdirectory_with_folder _folder_name _folder)
    add_subdirectory(${_folder} ${ARGN})
    set_property(DIRECTORY "${_folder}" PROPERTY FOLDER "${_folder_name}")
endfunction()

add_subdirectory_with_folder("thirdParty" glad)

option(GLFW_INSTALL "" OFF)
option(GLFW_BUILD_DOCS "" OFF)
add_subdirectory_with_folder("thirdParty/GLFW" glfw)

add_subdirectory_with_folder("thirdParty" imgui-1.78)

OPTION(BUILD_BULLET2_DEMOS "" OFF)
OPTION(BUILD_CLSOCKET "" OFF)
OPTION(BUILD_CPU_DEMOS "" OFF)
OPTION(BUILD_EXTRAS "" OFF)
OPTION(BUILD_OPENGL3_DEMOS "" OFF)
OPTION(BUILD_PYBULLET "" OFF)
OPTION(BUILD_SHARED_LIBS "" OFF)
OPTION(BUILD_UNIT_TESTS "" OFF)
OPTION(BULLET2_MULTITHREADING "" OFF)
OPTION(CLSOCKET_DEP_ONLY "" OFF)
OPTION(CLSOCKET_SHARED "" OFF)
#OPTION(INSTALL_CMAKE_FILES "" OFF)
OPTION(INSTALL_LIBS "" OFF)
OPTION(USE_GLUT ""	OFF)
OPTION(USE_GRAPHICAL_BENCHMARK "" OFF)
add_subdirectory_with_folder("thirdParty/Bullet" bullet3)

option(ALSOFT_EXAMPLES "" OFF)
option(ALSOFT_INSTALL "" OFF)
option(ALSOFT_UTILS "" OFF)
add_subdirectory_with_folder("thirdParty/OpenAL" openal-soft)

add_subdirectory_with_folder("thirdParty" dr_wav)

option(ASSIMP_BUILD_ASSIMP_TOOLS "" OFF)
option(ASSIMP_BUILD_TESTS "" OFF)
option(ASSIMP_BUILD_SAMPLES "" OFF)
option(ASSIMP_INSTALL "" OFF)
option(ASSIMP_INSTALL_PDB "" OFF)
add_subdirectory_with_folder("thirdParty/assimp" assimp)

add_subdirectory_with_folder("thirdParty" stb)
add_subdirectory_with_folder("thirdParty" debug-draw)
add_subdirectory_with_folder("thirdParty" whereami2cpp)

# We mantain tracy disabled by default. It needs to be enabled by the parent CMake file.
option(TRACY_ENABLE "" OFF)
option(TRACY_ON_DEMAND "" ON)
option(TRACY_NO_BROADCAST "" ON)
add_subdirectory_with_folder("thirdParty" tracy)