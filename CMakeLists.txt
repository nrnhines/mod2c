cmake_minimum_required(VERSION 2.8.9)
set(CMAKE_BUILD_TYPE Release)

project(mod2c)

set(MOD2C_DESCRIPTION "MOD2C converter")

set(VERSION_MAJOR 1)
set(VERSION_MINOR 1)
set(VERSION_PATCH 1)
set(VERSION_STRING "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

option(UNIT_TESTS "Enable unit tests compilation and execution" FALSE)
option(FUNCTIONAL_TESTS "Enable functional tests compilation and execution" FALSE)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake
      ${PROJECT_SOURCE_DIR}/CMake/config)

include(DefineInstallationPaths)
include(ReleaseDebugAutoFlags)

include(FindPkgConfig)
include(CTest)

find_package(BISON REQUIRED)
find_package(FLEX REQUIRED)

add_subdirectory(src/mod2c)
add_subdirectory(src/mod2c_core)
add_subdirectory(test/validation)

install(FILES share/nrnunits.lib DESTINATION share)
