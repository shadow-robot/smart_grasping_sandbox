cmake_minimum_required(VERSION 2.8.3)
project(fh_desc)
find_package(catkin REQUIRED COMPONENTS urdf xacro rospy)

catkin_python_setup()

include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS
    CATKIN_DEPENDS urdf xacro rospy
    INCLUDE_DIRS
    LIBRARIES
)


install(DIRECTORY common/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/common)

install(DIRECTORY hand/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/hand)

install(DIRECTORY meshes/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

install(DIRECTORY other/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/other)

install(DIRECTORY robots/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robots)

install(PROGRAMS scripts/fh_desc/urdf_loader.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


catkin_add_nosetests(test)
