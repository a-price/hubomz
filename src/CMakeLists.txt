# CMakeLists.txt for hubomz/src


add_library( hubomz fakerave.cpp fakerave_parse.cpp HuboPlus.cpp HuboKin.cpp )

add_executable( testkin testkin.cpp )
target_link_libraries( testkin hubomz mzcommon ${OPENGL_LIBRARY} expat )

add_executable( testzmp testzmp.cpp )
target_link_libraries( testzmp hubomz mzcommon ${OPENGL_LIBRARY} expat )

add_executable( testik testik.cpp )
target_link_libraries( testik hubomz mzcommon ${OPENGL_LIBRARY} expat )

add_executable( testcomik testcomik.cpp )
target_link_libraries( testcomik hubomz mzcommon ${OPENGL_LIBRARY} expat )

add_executable( reconcile reconcile.cpp )
target_link_libraries( reconcile hubomz mzcommon ${OPENGL_LIBRARY} expat )

add_gui_app( walkdemo walkdemo.cpp )
target_link_libraries( walkdemo hubomz mzcommon ${GLUT_LIBRARY} 
  ${OPENGL_LIBRARY} expat )
