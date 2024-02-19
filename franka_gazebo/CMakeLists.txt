cmake_minimum_required(VERSION 3.10.2)
project(franka_gazebo)


if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)

if("$ENV{IGNITION_VERSION}" STREQUAL "citadel")
  find_package(ignition-gazebo3 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo3_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Citadel")

elseif("$ENV{IGNITION_VERSION}" STREQUAL "edifice")
  find_package(ignition-gazebo5 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo5_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Edifice")

elseif("$ENV{IGNITION_VERSION}" STREQUAL "fortress")
  find_package(ignition-gazebo6 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Fortress")

else()
  find_package(ignition-gazebo6 REQUIRED)
  set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
  message(STATUS "Compiling against Ignition Fortress")
endif()

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
find_package(ignition-gazebo6 REQUIRED)

set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

include_directories(include)


add_library(${PROJECT_NAME}-plugin SHARED
  src/panda_anti_gravity_plugin.cpp
)

target_link_libraries(${PROJECT_NAME}-plugin
    PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
    PRIVATE ignition-gazebo6::ignition-gazebo6
)

ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_plugins)

install(TARGETS ${PROJECT_NAME}-plugin
  DESTINATION lib
)
  
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

pluginlib_export_plugin_description_file(
    ${PROJECT_NAME} ${PROJECT_NAME}_plugins.xml
)

ament_package()