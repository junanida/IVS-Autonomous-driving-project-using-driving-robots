cmake_minimum_required(VERSION 3.0.2)
project(aruco_pose_ros)

## C++17 표준을 사용하도록 설정
add_compile_options(-std=c++17)

## Catkin 필수 패키지 찾기
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  std_msgs
  tf2_ros
)

## OpenCV와 Python 라이브러리 찾기
find_package(OpenCV REQUIRED)
find_package(PythonLibs 3 REQUIRED)
find_package(CURL REQUIRED)

## Catkin 패키지 선언
catkin_package()

## 추가 헤더 파일 경로 지정
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}  # Python 헤더 파일 경로 추가
  ${CURL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/matplotlib-cpp
)

## 실행 파일 생성 및 링크
add_executable(aruco_pose_node src/aruco_pose_node.cpp)
target_link_libraries(aruco_pose_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PYTHON_LIBRARIES}  # Python 라이브러리 링크
  ${CURL_LIBRARIES}
  ##CURL::libcurl
)

add_executable(image_publisher_node src/image_publisher_node.cpp)
target_link_libraries(image_publisher_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


