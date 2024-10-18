cmake_minimum_required(VERSION 3.28)

project(VideoPlayer)

# name of application. replace 'app' with desired app name
set(APP_NAME allo_player)

# cmake scripts are kept in the cmake directory
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# disable in-source builds
include(NoInSourceBuilds)
# enable interprocedural optimization if possible
include(SetIPO)

if(MSVC)
  set(CMAKE_BUILD_PARALLEL_LEVEL $ENV{NUMBER_OF_PROCESSORS})
  # add_compile_options(/utf-8)
  # set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:4194304")
endif()

# path to main source file
# add_executable(${APP_NAME}
#                src/main.cpp
#                src/al_VideoApp.hpp
#                src/al_VideoApp.cpp
#                src/MTCReader.hpp
#                src/MTCReader.cpp)
# add_executable(${APP_NAME} src/test.cpp)
# add_executable(${APP_NAME} src/test2.cpp)
add_executable(${APP_NAME} src/test3.cpp)

add_subdirectory(allolib)
target_link_libraries(${APP_NAME} PRIVATE alapp)

if (EXISTS ${CMAKE_CURRENT_LIST_DIR}/al_ext)
  message("Buiding extensions in al_ext")
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/al_ext)
  get_target_property(AL_EXT_LIBRARIES al_ext AL_EXT_LIBRARIES)
  target_link_libraries(${APP_NAME} PRIVATE ${AL_EXT_LIBRARIES})
endif()

set_target_properties(${APP_NAME} PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin
  RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_LIST_DIR}/debug/bin
  RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_LIST_DIR}/bin
  # ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/lib
  # LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin
)