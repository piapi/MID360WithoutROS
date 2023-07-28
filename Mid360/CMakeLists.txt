

  cmake_minimum_required(VERSION 3.10.2)


  #---------------------------------------------------------------------------------------
  # Start livox_ros_driver2 project
  #---------------------------------------------------------------------------------------
  # include(cmake/version.cmake)
  project(mid360_drivers)
  # message(STATUS "driver version: ${LIVOX_ROS_DRIVER2_VERSION}")


  #---------------------------------------------------------------------------------------
  # find package and the dependecy
  #---------------------------------------------------------------------------------------
  find_package(Boost 1.54 REQUIRED COMPONENTS
    system
    thread
    chrono
  )


  ## Find pcl lib
  find_package(PCL REQUIRED)





  find_package(PkgConfig)
  pkg_check_modules(APR apr-1)
  if (APR_FOUND)
    message(${APR_INCLUDE_DIRS})
    message(${APR_LIBRARIES})
  endif (APR_FOUND)

  ###################################

  #---------------------------------------------------------------------------------------
  # Set default build to release
  #---------------------------------------------------------------------------------------
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose Release or Debug" FORCE)
  endif()

  #---------------------------------------------------------------------------------------
  # Compiler config
  #---------------------------------------------------------------------------------------
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)


  ## PCL library
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  include_directories(/usr/local/include)

  #---------------------------------------------------------------------------------------
  # generate excutable and add libraries
  #---------------------------------------------------------------------------------------
  add_executable(mid360_node src/LivoxMid360.cpp)

  # #---------------------------------------------------------------------------------------
  # # precompile macro and compile option
  # #---------------------------------------------------------------------------------------
  # target_compile_options(mid360_node
  #   PRIVATE $<$<CXX_COMPILER_ID:GNU>:-Wall>
  # )

  # #---------------------------------------------------------------------------------------
  # # add projects that depend on
  # #---------------------------------------------------------------------------------------

  #---------------------------------------------------------------------------------------
  # source file
  #---------------------------------------------------------------------------------------
  target_sources(mid360_node
    PRIVATE
    src/lds.cpp
    src/lds_lidar.cpp
    src/lddc.cpp
    src/comm/comm.cpp
    src/comm/ldq.cpp
    src/comm/semaphore.cpp
    src/comm/lidar_imu_data_queue.cpp
    src/comm/cache_index.cpp
    src/comm/pub_handler.cpp

    src/parse_cfg_file/parse_cfg_file.cpp
    src/parse_cfg_file/parse_livox_lidar_cfg.cpp

    src/call_back/lidar_common_callback.cpp
    src/call_back/livox_lidar_callback.cpp
  )

  #---------------------------------------------------------------------------------------
  # include file
  #---------------------------------------------------------------------------------------
  target_include_directories(mid360_node
    PUBLIC
    ${PCL_INCLUDE_DIRS}
    ${APR_INCLUDE_DIRS}
    /usr/local/include
    3rdparty
    src
  )

  #---------------------------------------------------------------------------------------
  # link libraries
  #---------------------------------------------------------------------------------------
  target_link_libraries(mid360_node
    ${Boost_LIBRARY}
    ${PCL_LIBRARIES}
    ${APR_LIBRARIES}
    /usr/local/lib/liblivox_lidar_sdk_shared.so
    -lpthread -lm
  )


  #---------------------------------------------------------------------------------------
  # Install
  #---------------------------------------------------------------------------------------

  install(TARGETS mid360_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
