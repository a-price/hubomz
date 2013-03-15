add_executable(zmpdemo zmpdemo.cpp zmpwalkgenerator.cpp footprint.cpp)
add_executable(swingtest swing-test.cpp swing.cpp)
add_executable(footprintdemo footprint-example.cpp footprint.cpp)


set(zmpdemo_libs hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat)


if (${HAVE_HUBO_ACH})
set(zmpdemo_libs ${zmpdemo_libs} ach)
endif()

target_link_libraries(zmpdemo ${zmpdemo_libs})


add_executable(gait-timer-test gait-timer-test.cpp gait-timer.cpp)
target_link_libraries(zmpdemo hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat ach)
