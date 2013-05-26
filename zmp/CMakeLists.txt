add_library(hubozmp zmpwalkgenerator.cpp footprint.cpp gait-timer.cpp swing.cpp)


add_gui_app(zmpdemo zmpdemo.cpp)
add_executable(swingtest swing-test.cpp)
add_executable(footprint-gnuplot footprint-gnuplot.cpp)
add_executable(footprint-opengl footprint-opengl.cpp)
add_executable(gait-timer-test gait-timer-test.cpp)


set(zmpdemo_libs hubozmp hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat)

if (${HAVE_HUBO_ACH})
set(zmpdemo_libs ${zmpdemo_libs} ach)
endif()

<<<<<<< HEAD
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
=======
target_link_libraries(zmpdemo             ${zmpdemo_libs})
target_link_libraries(footprint-gnuplot   ${zmpdemo_libs})
target_link_libraries(footprint-opengl    ${zmpdemo_libs})
target_link_libraries(swingtest           ${zmpdemo_libs})
target_link_libraries(gait-timer-test     ${zmpdemo_libs})
>>>>>>> 6dabe8f8e595d7f2fd822d9cd58dd629b763949f
