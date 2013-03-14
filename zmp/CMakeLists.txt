add_executable(zmpdemo zmpdemo.cpp)
add_executable(swingtest swing-test.cpp swing.cpp)
add_executable(gaittest gait-test.cpp gait.cpp)
target_link_libraries(zmpdemo hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat ach)
