cmake_minimum_required(VERSION 2.8.3)
project(windturb)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
)

find_package(VISP REQUIRED)
find_package( OpenCV REQUIRED )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES windturb
#  CATKIN_DEPENDS ok_tools
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
${VISP_INCLUDE_DIRS}
)



add_executable(windturb src/windturb.cpp include/windturb/okExperiment.h src/okExperiment.cpp)
target_link_libraries(windturb  ${catkin_LIBRARIES} ${VISP_LIBRARIES} ${OpenCV_LIBS} )
