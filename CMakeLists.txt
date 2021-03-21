cmake_minimum_required(VERSION 2.8.3)
project(manipulator_asif)
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)	

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf)
find_package(asif++ CONFIG REQUIRED)
find_package(pinocchio REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

catkin_package(
        DEPENDS
        pinocchio
        Boost
)

include_directories(include ${catkin_INCLUDE_DIRS}  /usr/local/include/eigen3 ${pinocchio_INCLUDE_DIRS})
link_directories(boost_system ${pinocchio_LIBRARY_DIRS})

add_executable(human src/human.cpp)
target_link_libraries(human ${catkin_LIBRARIES})

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES} boost_system urdfdom_model ${pinocchio_LIBRARIES})
target_compile_options(controller PUBLIC ${pinocchio_CFLAGS_OTHER})

add_executable(integrator src/integrator.cpp)
target_link_libraries(integrator ${catkin_LIBRARIES} boost_system urdfdom_model ${pinocchio_LIBRARIES})
target_compile_options(integrator PUBLIC ${pinocchio_CFLAGS_OTHER})

add_executable(task_giver src/task_giver.cpp)
target_link_libraries(task_giver ${catkin_LIBRARIES} boost_system urdfdom_model ${pinocchio_LIBRARIES})
target_compile_options(task_giver PUBLIC ${pinocchio_CFLAGS_OTHER})

add_executable(implicit_asif src/implicit_asif.cpp)
target_link_libraries(implicit_asif ${catkin_LIBRARIES} boost_system urdfdom_model asif++::asif++ ${pinocchio_LIBRARIES})
target_compile_options(implicit_asif PUBLIC ${pinocchio_CFLAGS_OTHER})

add_executable(kinematic_velocity_barrier src/kinematic_velocity_barrier.cpp)
target_link_libraries(kinematic_velocity_barrier ${catkin_LIBRARIES} boost_system urdfdom_model asif++::asif++ ${pinocchio_LIBRARIES})
target_compile_options(kinematic_velocity_barrier PUBLIC ${pinocchio_CFLAGS_OTHER})