add_library(hubozmp zmpwalkgenerator.cpp footprint.cpp gait-timer.cpp swing.cpp)


add_gui_app(zmpdemo zmpdemo.cpp)
add_gui_app(zmpgui zmpgui.cpp)
add_executable(zmp-daemon zmp-daemon.cpp)
add_executable(swingtest swing-test.cpp)
add_executable(footprint-gnuplot footprint-gnuplot.cpp)
add_executable(footprint-opengl footprint-opengl.cpp)
add_executable(gait-timer-test gait-timer-test.cpp)


set(zmpdemo_libs hubozmp hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat)
set(zmp-daemon_libs hubozmp hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat)
set(zmpgui_libs hubozmp hubomz mzcommon ${OPENGL_LIBRARY} ${GLUT_LIBRARY} expat)


if (${HAVE_HUBO_ACH})
set(zmpdemo_libs ${zmpdemo_libs} ach)
set(zmp-daemon_libs ${zmp-daemon_libs} ach)
set(zmpgui_libs ${zmpgui_libs} ach)
endif()

target_link_libraries(zmp-daemon          ${zmp-daemon_libs})
target_link_libraries(zmpdemo             ${zmpdemo_libs})
target_link_libraries(zmpgui              ${zmpgui_libs})
target_link_libraries(footprint-gnuplot   ${zmpdemo_libs})
target_link_libraries(footprint-opengl    ${zmpdemo_libs})
target_link_libraries(swingtest           ${zmpdemo_libs})
target_link_libraries(gait-timer-test     ${zmpdemo_libs})
