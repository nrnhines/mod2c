cmake_minimum_required(VERSION 2.8.9)
set(CMAKE_BUILD_TYPE Release)

project(mod2c)

set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)

set(MOD2C_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMake ${CMAKE_SOURCE_DIR}/CMake/common)
include(GitExternal)
include(Common)

# treat white-space separated string as a list
macro(wslist listcmd listvar)
    string(REGEX REPLACE "[ 	]+" ";" ${listvar} "${${listvar}}")
    list(${listcmd} ${listvar} ${ARGN})
    string(REPLACE ";" " " ${listvar} "${${listvar}}")
endmacro()

# legacy code; clean up is for the future
wslist(REMOVE_ITEM CMAKE_C_FLAGS "-Werror")
wslist(REMOVE_ITEM CMAKE_CXX_FLAGS "-Werror")

find_package(BISON REQUIRED)
find_package(FLEX REQUIRED)

#add_subdirectory(src/nocmodl)
add_subdirectory(src/nocmodl_core)
add_subdirectory(test/validation)

install(FILES share/nrnunits.lib DESTINATION share)
