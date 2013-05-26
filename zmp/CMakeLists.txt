add_executable(zmpdemo zmpdemo.cpp zmpwalkgenerator.cpp footprint.cpp gait-timer.cpp swing.cpp)
add_executable(swingtest swing-test.cpp swing.cpp)
add_executable(footprintdemo footprint-example.cpp footprint.cpp)


set(zmpdemo_libs hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat)

if (${HAVE_HUBO_ACH})
set(zmpdemo_libs ${zmpdemo_libs} ach)
endif()

target_link_libraries(zmpdemo ${zmpdemo_libs})


add_executable(gait-timer-test gait-timer-test.cpp gait-timer.cpp)
target_link_libraries(zmpdemo hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat ach)

add_executable(VectorViewer VectorViewer.cpp)
target_link_libraries(VectorViewer ${zmpdemo_libs})


find_package(Boost REQUIRED COMPONENTS system filesystem thread)
include_directories(${BOOST_INCLUDE_DIRS})
link_libraries(${BOOST_LIBRARIES})

set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
find_package(Eigen REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "-std=c++0x -g")
#add_definitions(-pg)
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")

add_executable(stlaio stla_io_prb.cpp stla_io.cpp)
add_executable(graphTest stl_1.cpp stla_io.cpp MeshSurface.cpp)
target_link_libraries(graphTest hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat ach)