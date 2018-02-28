# Install script for directory: /home/robot/Documents/DifferentialDrive/AprilTags

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/robot/Documents/DifferentialDrive/AprilTags/build/lib/libapriltags.a")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/AprilTags" TYPE FILE FILES
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/Edge.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/FloatImage.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/GLine2D.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/GLineSegment2D.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/Gaussian.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/GrayModel.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/Gridder.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/Homography33.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/MathUtil.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/Quad.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/Segment.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/Tag16h5.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/Tag16h5_other.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/Tag25h7.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/Tag25h9.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/Tag36h11.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/Tag36h11_other.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/Tag36h9.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/TagDetection.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/TagDetector.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/TagFamily.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/UnionFindSimple.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/XYWeight.h"
    "/home/robot/Documents/DifferentialDrive/AprilTags/AprilTags/pch.h"
    )
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/robot/Documents/DifferentialDrive/AprilTags/build/lib/pkgconfig/apriltags.pc")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/robot/Documents/DifferentialDrive/AprilTags/build/example/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/robot/Documents/DifferentialDrive/AprilTags/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
