cmake_minimum_required(VERSION 3.11)
project(ow_fk)

# Set the rpath (to build deb package)
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    ow_core
    ow_rbdl
)

set(DEPENDS_VAR system_lib)
if("$ENV{ROS_DISTRO}" STRGREATER "indigo")
    set(DEPENDS_VAR )
endif()

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ow_fk
    CATKIN_DEPENDS
        roscpp
        ow_core
        ow_rbdl
    DEPENDS ${DEPENDS_VAR}
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)



set(HEADER_FILES
    body_part.h
    forward_kinematics.h 
    robot_model.h
    urdf_loader.h
)

foreach(file ${HEADER_FILES})
    list(APPEND HEADERS include/${PROJECT_NAME}/${file})
endforeach()

set(SOURCES
    src/body_part.cpp
    src/forward_kinematics.cpp  
    src/robot_model.cpp
    src/urdf_loader.cpp
)

# add executables
add_library(ow_fk ${SOURCES} ${HEADERS})
add_dependencies(ow_fk
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})

# link target against libs
target_link_libraries(ow_fk
    ${catkin_LIBRARIES}
)

set(APP_TARGETS
    #ow_fk_test1
)

foreach(target ${APP_TARGETS})
        add_executable(${target} src/applications/main_${target}.cpp)
        add_dependencies(${target}
            ${${PROJECT_NAME}_EXPORTED_TARGETS}
            ${catkin_EXPORTED_TARGETS})

        target_link_libraries(${target}
            ow_fk
            ${catkin_LIBRARIES}
        )
endforeach()

######################################################################################
# TUM_ICS_NOTE: We need to modify the installation variables to be compliant with the
#   debtools. This function is controlled with the global variable -DDEB_BUILD
#   (true/false)
#include(/usr/share/cmake/CMakeTools/ics_tools.cmake)
#ics_ros_debgen()

#install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)

# don't install private headers
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN "*_p.h" EXCLUDE
    PATTERN "*.h"
)

install(TARGETS ow_fk ${APP_TARGETS}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
