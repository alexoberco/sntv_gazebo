cmake_minimum_required(VERSION 3.10)
project(sntv_gazebo VERSION 1.0.0 LANGUAGES C CXX)

# Builds inside a ROS 2 workspace (colcon, Gazebo from the gz_*_vendor packages) or as a plain
# CMake project against a system Gazebo Harmonic (gz-sim8, gz-rendering8).
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(gz_sim_vendor REQUIRED)
  find_package(gz_plugin_vendor REQUIRED)
  find_package(gz_rendering_vendor REQUIRED)
  find_package(gz_ogre_next_vendor QUIET)
endif()
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-rendering8 REQUIRED COMPONENTS ogre2)
find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)

# NVENC headers (Debian/Ubuntu: libffmpeg-nvenc-dev). The library, libnvidia-encode.so.1,
# comes with the NVIDIA driver and is loaded at run time.
find_path(NVENC_INCLUDE_DIR ffnvcodec/nvEncodeAPI.h REQUIRED)
# OGRE-Next: the plugin registers itself as an Ogre::FrameListener.
find_path(OGRE_NEXT_INCLUDE_DIR OgreRoot.h REQUIRED
  PATH_SUFFIXES OGRE-Next
  PATHS /opt/ros/$ENV{ROS_DISTRO}/opt/gz_ogre_next_vendor/include /usr/include /usr/local/include)
find_library(OGRE_NEXT_MAIN OgreNextMain REQUIRED
  PATHS /opt/ros/$ENV{ROS_DISTRO}/opt/gz_ogre_next_vendor/lib)

add_library(sntv_gazebo_system SHARED src/sntv_stream_system.cpp)
target_compile_features(sntv_gazebo_system PRIVATE cxx_std_17)
# Ogre warns on every file that it cannot tell a debug build; it assumes release, which is right.
target_compile_definitions(sntv_gazebo_system PRIVATE OGRE_IGNORE_UNKNOWN_DEBUG)
target_include_directories(sntv_gazebo_system PRIVATE include ${NVENC_INCLUDE_DIR} ${OGRE_NEXT_INCLUDE_DIR})
target_link_libraries(sntv_gazebo_system
  gz-sim8::gz-sim8 gz-plugin2::register gz-rendering8::gz-rendering8 gz-rendering8::ogre2
  ${OGRE_NEXT_MAIN} OpenGL::OpenGL OpenGL::EGL ${CMAKE_DL_LIBS})

install(TARGETS sntv_gazebo_system LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY examples DESTINATION share/${PROJECT_NAME})

if(ament_cmake_FOUND)
  # GZ_SIM_SYSTEM_PLUGIN_PATH for whoever sources this workspace.
  ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
  ament_package()
endif()
