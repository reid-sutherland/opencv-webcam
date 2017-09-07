cmake_minimum_required(VERSION 2.8)
project( opencv-webcam )

find_package( OpenCV REQUIRED )
find_package( PCL REQUIRED )

set(CMAKE_CXX_STANDARD 11)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

aux_source_directory(. SRC_LIST)
#add_executable( opencv-webcam main.cpp CaptureThread.h CaptureThread.cpp SharedImageBuffer.cpp SharedImageBuffer.h Buffer.h Config.h Structures.h ProcessingThread.h ProcessingThread.cpp CameraProperties.h UtilsCameras.h CameraView.cpp CameraView.h ${SRC_LIST})
add_executable( ${PROJECT_NAME} ${SRC_LIST} FaceDetection.cpp FaceDetection.h)
target_link_libraries( opencv-webcam ${OpenCV_LIBS} ${PCL_LIBRARIES} m rt pthread)
target_link_libraries( opencv-webcam -lX11 )