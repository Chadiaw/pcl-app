cmake_minimum_required( VERSION 3.6 )

# Create Project
project( pcl_app )

# Set StartUp Project
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "pcl_app" )

# Find PCL Package
set( PCL_ROOT $ENV{PCL_ROOT} )
find_package( PCL 1.8.0 REQUIRED )

# Find VTK Package
set( VTK_DIR "$ENV{PCL_ROOT}/3rdParty/VTK" )
find_package( VTK REQUIRED )

# Find Qt Package
set( QTDIR $ENV{QTDIR} )
set( CMAKE_PREFIX_PATH ${QTDIR} )
find_package( Qt5Widgets REQUIRED )

# Set Automatic MOC
set( CMAKE_INCLUDE_CURRENT_DIR ON )
set( CMAKE_AUTOMOC ON )

# Generate MOC Code and UI Header
qt5_wrap_cpp( MOC pclapp.h kinectgrabber.h )
qt5_wrap_ui( UI_HEADER pclapp.ui )

# Add Executable
add_executable( pcl_app main.cpp pclapp.h pclapp.cpp kinectgrabber.h kinectgrabber.cpp utils.h utils.cpp pclapp.ui )

# Additional Include Directories
include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# Load VTK Settings
include( ${VTK_USE_FILE} )

# Preprocessor Definitions
add_definitions( ${PCL_DEFINITIONS} )

# Additional Library Directories
link_directories( ${PCL_LIBRARY_DIRS} )

# Additional Dependencies
target_link_libraries( pcl_app ${PCL_LIBRARIES} ${VTK_LIBRARIES} Qt5::Widgets )
