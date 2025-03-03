cmake_minimum_required(VERSION 3.8)
project(keypoint_camera_pub)

if(CMAKE_COMPILE_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(CURL REQUIRED)
find_package(GDAL REQUIRED)
find_package(TIFF REQUIRED)
find_package(OrbbecSDK REQUIRED HINTS "/home/anas/Desktop/c_3rdparty/OrbbecSDK/build/")
find_package(slam_interfaces REQUIRED)
find_package(OpenCV 3.0 QUIET)

set(OrbbecSDK_INCLUDE /home/anas/Desktop/c_3rdparty/OrbbecSDK/build/install/include/)
set(OrbbecSDK_LIB_ORB /home/anas/Desktop/c_3rdparty/OrbbecSDK/build/install/lib/libOrbbecSDK.so)

include_directories(
include
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${slam_interfaces_SOURCE_DIR}/slam_interfaces/include
)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/lib)
add_library(${PROJECT_NAME} SHARED
  src/ORBextractor.cc
)

add_executable(depth_camera src/depth_camera.cpp)
include_directories(depth_camera ${OrbbecSDK_INCLUDE})
target_link_libraries(depth_camera ${OrbbecSDK_LIB_ORB} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME})
ament_target_dependencies(depth_camera rclcpp std_msgs sensor_msgs cv_bridge image_transport CURL GDAL TIFF slam_interfaces OpenCV)
target_include_directories(depth_camera PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)

install(TARGETS depth_camera DESTINATION lib/${PROJECT_NAME})

add_executable(intrinsic_camera src/intrinsic_camera.cpp)
include_directories(intrinsic_camera ${OrbbecSDK_INCLUDE})
target_link_libraries(intrinsic_camera ${OrbbecSDK_LIB_ORB} ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME})
ament_target_dependencies(intrinsic_camera rclcpp std_msgs sensor_msgs cv_bridge image_transport CURL GDAL TIFF slam_interfaces OpenCV)
target_include_directories(intrinsic_camera PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)

install(TARGETS intrinsic_camera DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	set(ament_cmake_copyright_FOUND TRUE)
	set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
