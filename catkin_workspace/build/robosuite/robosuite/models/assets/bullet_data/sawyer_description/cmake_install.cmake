# Install script for directory: /home/jerrywang/spa/catkin_workspace/src/robosuite/robosuite/models/assets/bullet_data/sawyer_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jerrywang/spa/catkin_workspace/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/jerrywang/spa/catkin_workspace/build/robosuite/robosuite/models/assets/bullet_data/sawyer_description/catkin_generated/installspace/sawyer_description.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sawyer_description/cmake" TYPE FILE FILES
    "/home/jerrywang/spa/catkin_workspace/build/robosuite/robosuite/models/assets/bullet_data/sawyer_description/catkin_generated/installspace/sawyer_descriptionConfig.cmake"
    "/home/jerrywang/spa/catkin_workspace/build/robosuite/robosuite/models/assets/bullet_data/sawyer_description/catkin_generated/installspace/sawyer_descriptionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sawyer_description" TYPE FILE FILES "/home/jerrywang/spa/catkin_workspace/src/robosuite/robosuite/models/assets/bullet_data/sawyer_description/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sawyer_description/config" TYPE DIRECTORY FILES "/home/jerrywang/spa/catkin_workspace/src/robosuite/robosuite/models/assets/bullet_data/sawyer_description/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sawyer_description/meshes" TYPE DIRECTORY FILES "/home/jerrywang/spa/catkin_workspace/src/robosuite/robosuite/models/assets/bullet_data/sawyer_description/meshes/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sawyer_description/params" TYPE DIRECTORY FILES "/home/jerrywang/spa/catkin_workspace/src/robosuite/robosuite/models/assets/bullet_data/sawyer_description/params/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sawyer_description/urdf" TYPE DIRECTORY FILES "/home/jerrywang/spa/catkin_workspace/src/robosuite/robosuite/models/assets/bullet_data/sawyer_description/urdf/")
endif()

