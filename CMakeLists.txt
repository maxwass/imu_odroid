cmake_minimum_required(VERSION 2.8)
project( test )


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
#LIST(APPEND CMAKE_MODULE_PATH "./Sensor") # for libusb-1.0

#find_package( OpenCV REQUIRED )
#find_package( libusb-1.0 REQUIRED )
#find_package( TurboJPEG REQUIRED )
#find_package( freenect2 REQUIRED )
#find_package( OpenCL )

#add_definitions(-DREG_OPENCL_FILE="Sensor/depth_registration.cl")
#add_definitions(-DEPTH_REG_OPENCL)

include_directories( /usr/include )
#include_directories( /usr/local/include/eigen3 )
#include_directories( /usr/local/include/libfreenect2/tinythread ) #idk
#include_directories( include )
#include_directories( libs/liblinear_incdec )


add_executable( test 
  imu.cpp motor.cpp motor_test.cpp
)
target_link_libraries( test -lpthread -lm  )
