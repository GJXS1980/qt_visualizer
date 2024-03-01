cmake_minimum_required(VERSION 3.5)

project(mecheye_qt_visualizer)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt6 QUIET COMPONENTS Concurrent OpenGL Widgets)
if (NOT Qt6_FOUND)
  find_package(Qt5 COMPONENTS Concurrent OpenGL Widgets REQUIRED)
  set(QTX Qt5)
else()
  set(QTX Qt6)
endif()

# 添加 .qrc 文件
qt5_add_resources(QRC_FILES
    mecheye_qt_visualizer.qrc
)


# Find PCL
find_package(PCL REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
  message(
    FATAL_ERROR
      "OpenCV not found. Please point OpenCV_DIR to the directory of your OpenCV installation (containing the file OpenCVConfig.cmake)."
  )
endif()

# Find MechEyeApi
if(CMAKE_HOST_WIN32)
  find_package(MechEyeApi REQUIRED CONFIG PATHS "$ENV{MECHEYE_DIR}/API")
  if(NOT MechEyeApi_FOUND)
    message(
      FATAL_ERROR "MechEyeApi not found. Please install MechEyeApi first.")
  endif()
elseif(CMAKE_HOST_UNIX)
  find_package(PkgConfig)
  if(NOT PkgConfig_FOUND)
    message(FATAL_ERROR "PkgConfig not found.")
  else()
    pkg_check_modules(MECHEYEAPI REQUIRED MechEyeApi)
    if(NOT MECHEYEAPI_FOUND)
      message(
        FATAL_ERROR "MechEyeApi not found. Please install MechEyeApi first.")
    endif()
  endif()
endif()

include_directories(${MECHEYEAPI_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${MECHEYEAPI_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(project_SOURCES main.cpp pclviewer.cpp)


add_executable(${PROJECT_NAME} ${project_SOURCES} ${QRC_FILES} )

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${QTX}::Widgets ${MECHEYEAPI_LIBRARIES} ${OpenCV_LIBS})

# 设置启动时的环境变量
set_target_properties(${PROJECT_NAME} PROPERTIES
    ENVIRONMENT "QT_OPENGL=software"
)

