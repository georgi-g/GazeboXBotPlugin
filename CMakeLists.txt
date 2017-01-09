project(GazeboXBotPlugin)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(XBotCoreModel REQUIRED)
find_package(XBotCore-interfaces REQUIRED)
find_package(boost QUIET)
find_package(sharedlibpp REQUIRED)

include_directories(include
                    ${GAZEBO_INCLUDE_DIRS} 
                    ${XBotCore-interfaces_INCLUDE_DIRS}
                    ${sharedlibpp_INCLUDE_DIRS}
                    )
link_directories(${GAZEBO_LIBRARY_DIRS} ) 

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# C++ 11
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


# GazeboXBotPlugin
add_library(GazeboXBotPlugin SHARED src/GazeboXBotPlugin.cpp)
target_link_libraries(GazeboXBotPlugin ${GAZEBO_LIBRARIES} 
                                       ${Boost_LIBRARIES} 
                                       ${XBotCoreModel_LIBRARIES}
                                       ${sharedlibpp_LIBRARIES}
                                       ${XBotCore-interfaces_LIBRARIES}
                                       yaml-cpp 
                                       )


install(TARGETS GazeboXBotPlugin RUNTIME LIBRARY DESTINATION lib)
install(DIRECTORY include/ 
        DESTINATION include
        FILES_MATCHING PATTERN "*.h*")
