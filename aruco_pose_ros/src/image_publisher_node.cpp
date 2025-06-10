#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_publisher_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/image_raw", 1);

    // ArUco 마커 이미지를 읽어옴
    cv::Mat image = cv::imread("/home/lkh/catkin_ws/src/aruco_pose_ros/src/aruco_example3.png", cv::IMREAD_COLOR);
    if (image.empty()) {
        ROS_ERROR("Could not read the image");
        return 1;
    }

    ros::Rate loop_rate(5);
    while (nh.ok()) {
        // OpenCV 이미지를 ROS 이미지 메시지로 변환
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

        // 토픽으로 이미지 퍼블리시
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}