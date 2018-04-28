# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_BUILD_TYPE Release) 
project(ndt_fuser)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  rosconsole
  cmake_modules 
  ndt_registration 
  ndt_map 
  laser_geometry 
  sensor_msgs 
  visualization_msgs 
  ndt_visualisation 
  std_srvs 
#  message_generation
)
#add_definitions(-DNO_NDT_VIZ)
catkin_package(
    DEPENDS eigen libpcl-all-dev opengl 
    CATKIN_DEPENDS ndt_registration ndt_map laser_geometry sensor_msgs ndt_visualisation std_srvs message_runtime visualization_msgs
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} 
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

find_package(OpenGL REQUIRED)

include_directories(${OPENGL_INCLUDE_DIR})

find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")


include_directories(${Boost_INCLUDE_DIRS})

set (${PROJECT_NAME}_LIB_SRCS
#	    src/ndt_fuser.cpp
	    src/ndt_fuser_hmt.cpp
	)


add_library(${PROJECT_NAME} ${${PROJECT_NAME}_LIB_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(ndt_fuser_node src/ndt_fuser_node.cpp)
add_dependencies(ndt_fuser_node ndt_fuser__generate_messages_cpp)

target_link_libraries(ndt_fuser_node ${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OPENGL_LIBRARIES} )

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
install(TARGETS ndt_fuser_node ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
       )
install(FILES
   launch/fuser_2dlaser.launch  
   launch/fuser_hmt_vellodyne32.launch  
   launch/fuser_velodyne32.launch  
   launch/velo32.yaml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY maps/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
