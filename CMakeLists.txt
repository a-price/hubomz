# CMakeLists.txt for hubomz
cmake_minimum_required(VERSION 2.6)

project(HUBOMZ)

include(mzsettings.cmake)

add_subdirectory(mzcommon)
add_subdirectory(src)

include_directories(src)

add_subdirectory(zmp)

if (QT_FOUND)
add_subdirectory(calibrate)
endif()


#
# TESTING
#

enable_testing()
add_subdirectory(gtest-1.6.0)

add_executable(footprint-test zmp/footprint-test.cpp zmp/footprint.cpp )
target_link_libraries(footprint-test gtest gtest_main)

add_test(footprint-test footprint-test)
