include_directories(${QT_INCLUDE_DIR})

qt4_wrap_ui(calib_ui_headers CalibMainWindow.ui) 
qt4_wrap_cpp(calib_headers CalibMainWindow.h HuboWidget.h)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(calibrate_srcs
calibrate.cpp HuboDataModel.cpp ${calib_headers} ${calib_ui_headers} CalibMainWindow.cpp HuboWidget.cpp)

set(calibrate_libs ${QT_LIBRARIES} hubomz mzcommon ${OPENGL_LIBRARY} expat)


if (${HAVE_HUBO_ACH})
set(calibrate_srcs ${calibrate_srcs} HuboAchDriver.cpp)
set(calibrate_libs ${calibrate_libs} huboparams ach)
endif (${HAVE_HUBO_ACH})

add_gui_app(calibrate_gui ${calibrate_srcs})
target_link_libraries(calibrate_gui ${calibrate_libs})

