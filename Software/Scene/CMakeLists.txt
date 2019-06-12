cmake_minimum_required( VERSION 3.5.1 )
cmake_policy(VERSION 2.8)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/Scene/cmake/modules/")

link_directories( ${CMAKE_SOURCE_DIR}/lib )

list(APPEND CMAKE_CXX_FLAGS "-std=c++11")

# find the required packages
find_package(GLM REQUIRED)
message(STATUS "GLM included at ${GLM_INCLUDE_DIR}")
find_package(GLFW3 REQUIRED)
message(STATUS "Found GLFW3 in ${GLFW3_INCLUDE_DIR}")
find_package(ASSIMP REQUIRED)
message(STATUS "Found ASSIMP in ${ASSIMP_INCLUDE_DIR}")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
find_package(OpenGL REQUIRED)
add_definitions(${OPENGL_DEFINITIONS})
find_package(X11 REQUIRED)
# note that the order is important for setting the libs
# use pkg-config --libs $(pkg-config --print-requires --print-requires-private glfw3) in a terminal to confirm
set(SCENELIBS ${GLFW3_LIBRARY} X11 Xrandr Xinerama Xi Xxf86vm Xcursor GL pthread ${ASSIMP_LIBRARY})

#configure_file(configuration/root_directory.h.in configuration/root_directory.h)
#include_directories(${CMAKE_BINARY_DIR}/Scene/configuration)

# first create relevant static libraries required for other projects
add_library(STB_IMAGE "src/stb_image.cpp")
#set(SCENELIBS ${SCENELIBS} STB_IMAGE)

add_library(GLAD "src/glad.c")
#set(SCENELIBS ${SCENELIBS} GLAD)

macro(makeLink src dest target)
  add_custom_command(TARGET ${target} POST_BUILD COMMAND ${CMAKE_COMMAND} -E create_symlink ${src} ${dest}  DEPENDS  ${dest} COMMENT "mklink ${src} -> ${dest}")
endmacro()

# copy shader files to build directory
file(GLOB SHADERS
         "src/*.vs"
         "src/*.fs"
)
foreach(SHADER ${SHADERS})
    file(COPY ${SHADER} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/..)
endforeach(SHADER)

#include_directories(${CMAKE_SOURCE_DIR}/Scene/includes)
#link_directories(${CMAKE_SOURCE_DIR}/Scene/includes)

add_library( WindowScene src/windowscene.cpp src/callbacks.cpp)
target_link_libraries( WindowScene ${SCENELIBS} GLAD STB_IMAGE dl)
#target_link_libraries( WindowScene ${SCENELIBS} GLAD STB_IMAGE)

