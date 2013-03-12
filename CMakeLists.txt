# CMakeLists.txt for hubomz
cmake_minimum_required(VERSION 2.6)

project(HUBOMZ)

include(mzsettings.cmake)

add_subdirectory(mzcommon)
add_subdirectory(src)

include_directories(src)

if (QT_FOUND)
add_subdirectory(calibrate)
endif()

