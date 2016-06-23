# Install script for directory: /home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/opt/android/ndk/android-ndk-r10e/toolchains/arm-linux-androideabi-4.9/prebuilt/linux-x86_64/user")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/build/lib/libumfpack.a")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/suitesparse" TYPE FILE FILES
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_col_to_triplet.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_defaults.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_free_numeric.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_free_symbolic.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_get_determinant.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_get_lunz.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_get_numeric.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_get_symbolic.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_global.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_load_numeric.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_load_symbolic.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_numeric.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_qsymbolic.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_report_control.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_report_info.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_report_matrix.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_report_numeric.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_report_perm.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_report_status.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_report_symbolic.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_report_triplet.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_report_vector.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_save_numeric.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_save_symbolic.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_scale.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_solve.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_symbolic.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_tictoc.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_timer.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_transpose.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_triplet_to_col.h"
    "/home/sarthak/Desktop/ORB-SLAM-Android/slam_ext/build.android/suitesparse/SuiteSparse/UMFPACK/Include/umfpack_wsolve.h"
    )
endif()

