cmake_minimum_required(VERSION 2.8.9)

project(mod2c)

set(CMAKE_BUILD_TYPE Release)

set(VERSION_MAJOR 1)
set(VERSION_MINOR 0)
set(VERSION_PATCH 0)

set(MOD2C_VERSION "${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_PATCH}")

find_package(BISON REQUIRED)
find_package(FLEX REQUIRED)

#add_subdirectory(src/nocmodl)
add_subdirectory(src/nocmodl_core)

