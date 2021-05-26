cmake_minimum_required(VERSION 2.8.3)
project(pub_sub)

# source ./devel/setup.bash
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    rospy 
    std_msgs 
    genmsg
    message_generation
)

## Declare ROS messages and services
add_message_files(DIRECTORY msg FILES Inputmsg.msg Outputmsg.msg)
# add_service_files(DIRECTORY srv FILES AddTwoInts.srv)
 
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)
 
## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS src/scripts/pub.py src/scripts/sub.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Build talker and listener
# include_directories(include ${catkin_INCLUDE_DIRS})
# add_executable(talker src/pub.cpp)
# target_link_libraries(talker ${catkin_LIBRARIES})
# add_dependencies(talker beginner_tutorials_generate_messages_cpp)

# add_executable(listener src/sub.cpp)
# target_link_libraries(listener ${catkin_LIBRARIES})