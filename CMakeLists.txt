cmake_minimum_required(VERSION 3.13)
project(DE)



#SET IMPORTANT PARAMETERS
#set(ZED_DIR "/usr/local/zed")
set(CMAKE_CXX_STANDARD 11)
set( ZED_DIR "/usr/local/zed/")




#FIND ALL THE PACKAGES REQUIRED

find_package(OpenCV REQUIRED )
find_package(zed REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED) # based on this tutorial https://github.com/stereolabs/zed-examples/blob/master/tutorials/tutorial%203%20-%20depth%20sensing/CMakeLists.txt





#INCLUDE IMPORTANT DIRECTORIES
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CUDA_INCLUDE_DIRS}) # for ZED
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/eigen/include/eigen3 ) #setting up eigen include (Eigen is template based lib)
# .. my includes (or wrappers over the top)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ) # my custom interfaces
# .. Adding ZED only libraries, (they depend on cuda, include them prior)





#CREATE A LIBRARY
link_directories(${ZED_LIBARY_DIR})
add_library(myLib SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/depth_estimator.cpp ) # appends file from src and creates a shared object
target_link_libraries(myLib ${OpenCV_LIBS})





#CREATEA EXECUTABLE

add_executable(DE src/main.cpp)


#LINK EXECUTABLE WITH CUSTOM BINARIES/LIBS

target_link_libraries(DE myLib)
target_link_libraries(DE ${ZED_LIBRARY_DIR})
target_link_libraries(DE ${CUDA_LIBARY_DIR})



# INSTALL EXEC PROPERTIES


# ...............NONE ..................




