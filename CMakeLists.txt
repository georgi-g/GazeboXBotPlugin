project(GazeboXBotPlugin)
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(kdl_parser REQUIRED)

find_package(boost QUIET)

include_directories(${GAZEBO_INCLUDE_DIRS} 
                    ${kdl_parser_INCLUDE_DIRS}
                    include
                    /home/lucamuratore/src/local_install/include) #TBD do it from superbuild
link_directories(${GAZEBO_LIBRARY_DIRS} ) #TBD do it from superbuild
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(kdl_parser REQUIRED)

# add_library(hello_world SHARED src/hello_world.cpp)
# target_link_libraries(hello_world ${GAZEBO_LIBRARIES})
# 
# add_library(model_push SHARED src/model_push.cpp)
# target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
# 
# add_library(system_gui SHARED src/system_gui.cpp)
# target_link_libraries(system_gui ${GAZEBO_LIBRARIES})

# # GazeboXBotPlugin
add_library(GazeboXBotPlugin SHARED src/GazeboXBotPlugin.cpp)
target_link_libraries(GazeboXBotPlugin ${GAZEBO_LIBRARIES} 
                                       ${Boost_LIBRARIES} 
                                       ${kdl_parser_LIBRARIES}
                                       yaml-cpp 
                                       /home/lucamuratore/src/local_install/lib/libXBotCoreModel.so)


install(TARGETS GazeboXBotPlugin RUNTIME LIBRARY DESTINATION lib)
install(DIRECTORY include/ 
        DESTINATION include
        FILES_MATCHING PATTERN "*.h*")