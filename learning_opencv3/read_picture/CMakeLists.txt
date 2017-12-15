cmake_minimum_required(VERSION 2.8)

project( examples )

find_package( OpenCV REQUIRED )
find_package( OpenGL )

if(OPENGL_FOUND)
  include_directories( ${OpenCV_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS} )
else()
  include_directories( ${OpenCV_INCLUDE_DIRS} )
endif()

#########################
# FOR LEARNING OPENCV 3 #
#########################

###############
### COMPILE ###
###############

# COMPILE EXAMPLES
add_executable( example_02-01 example_02-01.cpp )
add_executable( example_02-02 example_02-02.cpp )

################
###   LINK   ###
################
# LINK EXAMPLES:
target_link_libraries( example_02-01 ${OpenCV_LIBS} )
target_link_libraries( example_02-02 ${OpenCV_LIBS} )
