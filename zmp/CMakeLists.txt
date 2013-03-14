add_executable(zmpdemo zmpdemo.cpp)
add_executable(swingtest swing-test.cpp swing.cpp)

set(zmpdemo_libs hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat)


if (${HAVE_HUBO_ACH})
set(zmpdemo_libs ${zmpdemo_libs} ach)
endif()


target_link_libraries(zmpdemo ${zmpdemo_libs})

