#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "matplotlibcpp.h"
#include <string> 
#include <curl/curl.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace plt = matplotlibcpp;

class ArucoPoseNode {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    tf2_ros::TransformBroadcaster br_;
    ros::Publisher marker_pub_;  // Rviz에 시각화를 위한 퍼블리셔
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Mat cameraMatrix_, distCoeffs_;
    tf2_ros::TransformBroadcaster map_br_;
    ros::Timer timer_;

    geometry_msgs::TransformStamped global_transform;
    
    std::vector<double> marker23_x, marker23_y, marker23_z;
    std::vector<double> marker42_x, marker42_y, marker42_z;
    std::vector<double> marker84_x, marker84_y, marker84_z;
    std::vector<double> marker101_x, marker101_y, marker101_z;
    std::vector<double> marker156_x, marker156_y, marker156_z;  // 로봇 위치를 위한 벡터

    double marker23_x_fixed = 0.0, marker23_y_fixed = 0.0, marker23_z_fixed = 0.0;
    double marker42_x_fixed = 0.0, marker42_y_fixed = 0.0, marker42_z_fixed = 0.0;
    double marker84_x_fixed = 0.0, marker84_y_fixed = 0.0, marker84_z_fixed = 0.0;
    double marker101_x_fixed = 0.0, marker101_y_fixed = 0.0, marker101_z_fixed = 0.0;
    double marker156_x_fixed = 0.0, marker156_y_fixed = 0.0, marker156_z_fixed = 0.0;  // 로봇 위치의 고정된 값

    bool is_marker23_fixed = false;
    bool is_marker42_fixed = false;
    bool is_marker84_fixed = false;
    bool is_marker101_fixed = false;
    bool is_marker156_fixed = false;  // 로봇 위치의 고정 여부

    // 픽셀 좌표계에서의 마커 좌표를 저장하는 벡터들
    std::vector<double> pixel_marker23_x, pixel_marker23_y;
    std::vector<double> pixel_marker42_x, pixel_marker42_y;
    std::vector<double> pixel_marker84_x, pixel_marker84_y;
    std::vector<double> pixel_marker101_x, pixel_marker101_y;

    // 픽셀 좌표계에서의 고정된 좌표
    double pixel_marker23_x_fixed = 0.0, pixel_marker23_y_fixed = 0.0;
    double pixel_marker42_x_fixed = 0.0, pixel_marker42_y_fixed = 0.0;
    double pixel_marker84_x_fixed = 0.0, pixel_marker84_y_fixed = 0.0;
    double pixel_marker101_x_fixed = 0.0, pixel_marker101_y_fixed = 0.0;

    bool pixel_is_marker23_fixed = false;
    bool pixel_is_marker42_fixed = false;
    bool pixel_is_marker84_fixed = false;
    bool pixel_is_marker101_fixed = false;
    bool pixel_is_marker156_fixed = false;

    // 호모그래피 변환 행렬
    cv::Mat transformation_matrix;
    bool is_transformed = false;

    const int sample_count = 100;

public:
    ArucoPoseNode() : it_(nh_) {
        image_transport::TransportHints hints("compressed");
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ArucoPoseNode::imageCallback, this, hints);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("aruco_markers", 1);  // MarkerArray 퍼블리셔
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cameraMatrix_ = (cv::Mat_<double>(3, 3) << 1158.852896, 0, 954.737541, 0, 1165.296329, 528.07814, 0, 0, 1);
        distCoeffs_ = (cv::Mat_<double>(5, 1) << 0.073832, -0.167894, -0.000071, 0.002498, 0.000000);
        // timer_ = nh_.createTimer(ros::Duration(0.05), &ArucoPoseNode::publishMapTransform, this);
    }

    void publishMapTransform() {
        // 기존 map -> camera TF
        geometry_msgs::TransformStamped map_transform;
        map_transform.header.stamp = ros::Time::now();
        map_transform.header.frame_id = "map";
        map_transform.child_frame_id = "camera";
        map_transform.transform.translation.x = 0.0;
        map_transform.transform.translation.y = 0.0;
        map_transform.transform.translation.z = 0.0;
        map_transform.transform.rotation.x = 0.0;
        map_transform.transform.rotation.y = 0.0;
        map_transform.transform.rotation.z = 0.0;
        map_transform.transform.rotation.w = 1.0;

        map_br_.sendTransform(map_transform);

        // 마커들의 위치가 고정되면 Rviz에 시각화 //  && is_marker42_fixed && is_marker84_fixed && is_marker101_fixed
        if (is_marker23_fixed) {
            publishGlobalFrame();
            projectMarkersOntoGlobalFrame();
            logMarkersInGlobalFrame(); 
            publishMarkers();
            visualizeRobotInGlobalFrame();  // 로봇 위치 시각화
        }
        if (is_transformed){
            human_Pixel2Global();
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        // if (is_marker23_fixed && is_marker42_fixed && is_marker84_fixed && is_marker101_fixed) {
        //     return; // 모든 마커가 고정되었으면 더 이상 콜백을 수행하지 않음
        // }
        
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_ptr->image;
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::aruco::detectMarkers(image, dictionary_, markerCorners, markerIds);

        if (!markerIds.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix_, distCoeffs_, rvecs, tvecs);

            for (int i = 0; i < markerIds.size(); ++i) {
                int markerId = markerIds[i];

                // 각 마커의 코너 좌표 (픽셀 좌표계에서)
                std::vector<cv::Point2f> corners = markerCorners[i];

                // 마커의 중심점 계산 (4개의 코너의 평균)
                cv::Point2f center(0.0f, 0.0f);
                for (size_t j = 0; j < corners.size(); ++j) {
                    center.x += corners[j].x;
                    center.y += corners[j].y;
                }
                center.x /= corners.size();
                center.y /= corners.size();

                // 마커의 중심 좌표 출력
                // ROS_INFO("Marker %d Center in Pixel Coordinates: [x: %f, y: %f]", markerId, center.x, center.y);

                // Pixel Frame 평균 내기
                // if (markerId == 23 && !pixel_is_marker23_fixed) {
                //     collectSamples_Pixel(pixel_marker23_x, pixel_marker23_y, center.x, center.y, pixel_is_marker23_fixed, pixel_marker23_x_fixed, pixel_marker23_y_fixed);
    
                //     // float x23 = marker23_x_fixed * 100;
                //     // float y23 = marker23_y_fixed * 100;
                //     // send_pose_to_server23(x23,y23);

                // }

                if (markerId == 42 && !pixel_is_marker42_fixed) {
                    collectSamples_Pixel(pixel_marker42_x, pixel_marker42_y, center.x, center.y, pixel_is_marker42_fixed, pixel_marker42_x_fixed, pixel_marker42_y_fixed);
                    //int x42 = marker42_x_fixed;
                    //int y42 = marker42_y_fixed;
                    //send_pose_to_server42(marker42_x_fixed, marker42_y_fixed);
                }

                if (markerId == 84 && !pixel_is_marker84_fixed) {
                    collectSamples_Pixel(pixel_marker84_x, pixel_marker84_y, center.x, center.y, pixel_is_marker84_fixed, pixel_marker84_x_fixed, pixel_marker84_y_fixed);
                    //int x84 = marker42_x_fixed;
                    //int y84 = marker42_y_fixed;
                    //send_pose_to_server84(marker84_x_fixed, marker84_y_fixed);
                }

                // if (markerId == 101 && !pixel_is_marker101_fixed) {
                //     collectSamples_Pixel(pixel_marker101_x, pixel_marker101_y, center.x, center.y, pixel_is_marker101_fixed, pixel_marker101_x_fixed, pixel_marker101_y_fixed);
                //     //int x101 = marker42_x_fixed;
                //     //int y101 = marker42_y_fixed;
                //     //send_pose_to_server101(marker101_x_fixed, marker101_y_fixed);
                // }



                // Global Frame 평균 내기
                // if (markerId == 23 && !is_marker23_fixed) {
                //     collectSamples(marker23_x, marker23_y, marker23_z, tvecs[i][0], tvecs[i][1], tvecs[i][2], is_marker23_fixed, marker23_x_fixed, marker23_y_fixed, marker23_z_fixed);
                //     float x23 = marker23_x_fixed * 100;
                //     float y23 = marker23_y_fixed * 100;
                //     send_pose_to_server23(x23,y23);
                // }
                if (markerId == 42 && !is_marker42_fixed) {
                    collectSamples(marker42_x, marker42_y, marker42_z, tvecs[i][0], tvecs[i][1], tvecs[i][2], is_marker42_fixed, marker42_x_fixed, marker42_y_fixed, marker42_z_fixed);
                }
                if (markerId == 84 && !is_marker84_fixed) {
                    collectSamples(marker84_x, marker84_y, marker84_z, tvecs[i][0], tvecs[i][1], tvecs[i][2], is_marker84_fixed, marker84_x_fixed, marker84_y_fixed, marker84_z_fixed);
                }
                // if (markerId == 101 && !is_marker101_fixed) {
                //     collectSamples(marker101_x, marker101_y, marker101_z, tvecs[i][0], tvecs[i][1], tvecs[i][2], is_marker101_fixed, marker101_x_fixed, marker101_y_fixed, marker101_z_fixed);
                // }
                
                if (markerId == 23) {
                    marker23_x_fixed = tvecs[i][0];
                    marker23_y_fixed = tvecs[i][1];
                    marker23_z_fixed = tvecs[i][2];

                    visualizeRobotInGlobalFrame();
                    float x23 = marker23_x_fixed * 100;
                    float y23 = marker23_y_fixed * 100;
                    send_pose_to_server23(x23,y23);
                }

                if (markerId == 101) {
                    marker101_x_fixed = tvecs[i][0];
                    marker101_y_fixed = tvecs[i][1];
                    marker101_z_fixed = tvecs[i][2];

                    visualizeRobotInGlobalFrame();
                    float x101 = marker101_x_fixed * 100;
                    float y101 = marker101_y_fixed * 100;
                    send_pose_to_server101(x101,y101);
                }
                // 마커 156은 실시간으로 위치 업데이트 -> 파란색 구형
                if (markerId == 156) {
                    marker156_x_fixed = tvecs[i][0];
                    marker156_y_fixed = tvecs[i][1];
                    marker156_z_fixed = tvecs[i][2];

                    // 실시간으로 로봇 위치 시각화
                    visualizeRobotInGlobalFrame();
                    float x = marker156_x_fixed * 100;
                    float y = marker156_y_fixed * 100;
                    send_pose_to_server156(x,y);
                }
                // 모든 마커가 고정된 후에 Similiar transform 변환을 계산
                if (!is_transformed && pixel_is_marker23_fixed && pixel_is_marker42_fixed && pixel_is_marker84_fixed && pixel_is_marker101_fixed && is_marker23_fixed && is_marker42_fixed && is_marker84_fixed && is_marker101_fixed) {
                    ROS_INFO("Marker 23 in Pixel Frame: [x: %f, y: %f]", pixel_marker23_x_fixed, pixel_marker23_y_fixed);
                    ROS_INFO("Marker 42 in Pixel Frame: [x: %f, y: %f]", pixel_marker42_x_fixed, pixel_marker42_y_fixed);
                    ROS_INFO("Marker 84 in Pixel Frame: [x: %f, y: %f]", pixel_marker84_x_fixed, pixel_marker84_y_fixed);
                    ROS_INFO("Marker 101 in Pixel Frame: [x: %f, y: %f]", pixel_marker101_x_fixed, pixel_marker101_y_fixed);

                    pixel_marker23_x_fixed = pixel_marker23_x_fixed - pixel_marker101_x_fixed;
                    pixel_marker23_y_fixed = pixel_marker23_y_fixed - pixel_marker101_y_fixed;
                    pixel_marker42_x_fixed = pixel_marker42_x_fixed - pixel_marker101_x_fixed;
                    pixel_marker42_y_fixed = pixel_marker42_y_fixed - pixel_marker101_y_fixed;
                    pixel_marker84_x_fixed = (1920.0 - pixel_marker84_x_fixed) - (1920.0 - pixel_marker101_x_fixed);
                    pixel_marker84_y_fixed = pixel_marker84_y_fixed - pixel_marker101_y_fixed;
                    pixel_marker101_x_fixed = 0.0;
                    pixel_marker101_y_fixed = 0.0;
                    ROS_WARN("-----------------AFTER TRANSFORM------------------------");
                    ROS_INFO("Marker 23 in Pixel Frame: [x: %f, y: %f]", pixel_marker23_x_fixed, pixel_marker23_y_fixed);
                    ROS_INFO("Marker 42 in Pixel Frame: [x: %f, y: %f]", pixel_marker42_x_fixed, pixel_marker42_y_fixed);
                    ROS_INFO("Marker 84 in Pixel Frame: [x: %f, y: %f]", pixel_marker84_x_fixed, pixel_marker84_y_fixed);
                    ROS_INFO("Marker 101 in Pixel Frame: [x: %f, y: %f]", pixel_marker101_x_fixed, pixel_marker101_y_fixed);

                    // 픽셀 좌표계에서의 고정된 좌표
                    std::vector<cv::Point2f> pixel_points = {
                        cv::Point2f(pixel_marker23_x_fixed, pixel_marker23_y_fixed),
                        cv::Point2f(pixel_marker42_x_fixed, pixel_marker42_y_fixed),
                        cv::Point2f(pixel_marker84_x_fixed, pixel_marker84_y_fixed),
                        cv::Point2f(pixel_marker101_x_fixed, pixel_marker101_y_fixed)  // 마커 101은 기준이므로 원점 역할
                    };

                    // 글로벌 좌표계에서의 고정된 좌표 (마커 101을 원점으로 설정)
                    std::vector<cv::Point2f> global_points = {
                        cv::Point2f(marker23_x_fixed - marker101_x_fixed, marker23_y_fixed - marker101_y_fixed),  // Marker 23
                        cv::Point2f(marker42_x_fixed - marker101_x_fixed, marker42_y_fixed - marker101_y_fixed),  // Marker 42
                        cv::Point2f(marker84_x_fixed - marker101_x_fixed, marker84_y_fixed - marker101_y_fixed),  // Marker 84
                        cv::Point2f(0.0, 0.0)  // 마커 101을 원점으로 설정 (0,0)
                    };

                    // 1. 픽셀 좌표계와 글로벌 좌표계 사이의 스케일 및 회전 계산
                    cv::Point2f pixel_vector1 = pixel_points[0] - pixel_points[3];  // 픽셀 좌표에서 벡터
                    cv::Point2f pixel_vector2 = pixel_points[1] - pixel_points[3];  // 두 번째 벡터

                    cv::Point2f global_vector1 = global_points[0] - global_points[3];  // 글로벌 좌표에서 벡터
                    cv::Point2f global_vector2 = global_points[1] - global_points[3];  // 두 번째 벡터

                    // 2. 스케일 계산: 두 좌표계에서의 벡터 길이 비율을 통해 스케일을 계산
                    double scale_pixel = std::sqrt(pixel_vector1.x * pixel_vector1.x + pixel_vector1.y * pixel_vector1.y);
                    double scale_global = std::sqrt(global_vector1.x * global_vector1.x + global_vector1.y * global_vector1.y);
                    double s = scale_global / scale_pixel;

                    // 3. 회전 각도 계산: 두 좌표계의 벡터들 사이의 각도 계산
                    double theta_pixel = std::atan2(pixel_vector1.y, pixel_vector1.x);
                    double theta_global = std::atan2(global_vector1.y, global_vector1.x);
                    double theta = theta_global - theta_pixel;  // 글로벌 좌표계와 픽셀 좌표계 사이의 각도 차

                    // 4. 평행 이동 계산: 마커 101을 기준으로 평행 이동 (마커 101을 원점으로 설정했으므로 단순히 글로벌 좌표로 설정)
                    double t_x = global_points[3].x;
                    double t_y = global_points[3].y;

                    // 5. 유사 변환 행렬 생성 (스케일, 회전, 평행 이동)
                    transformation_matrix = (cv::Mat_<double>(3, 3) << 
                        s * std::cos(theta), -s * std::sin(theta), t_x,
                        s * std::sin(theta), s * std::cos(theta), t_y,
                        0, 0, 1);

                    // 변환 행렬을 로그로 출력
                    ROS_INFO_STREAM("Transformation matrix calculated (with Marker 101 as origin): \n" << transformation_matrix);

                    is_transformed = true;  // 변환이 완료되었음을 나타내기 위해 플래그 설정

                    
                }
                

                cv::aruco::drawAxis(image, cameraMatrix_, distCoeffs_, rvecs[i], tvecs[i], 0.1);
            }
        }

        cv::imshow("Image", image);
        cv::waitKey(1);
    }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void send_pose_to_server(float x, float y) {
        json data;
        data["x"] = x;
        data["y"] = y;
        std::string json_str = data.dump();

        CURL* curl = curl_easy_init();
        if (curl) {
            const std::string url = "http://192.168.202.66:5000/report_position";  // ← 여기에 Flask 서버 IP

            struct curl_slist* headers = nullptr;
            headers = curl_slist_append(headers, "Content-Type: application/json");

            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_str.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "curl failed: " << curl_easy_strerror(res) << std::endl;
            } else {
                //std::cout << "좌표 전송 완료: " << json_str << std::endl;
                std::cout << std::fixed << std::setprecision(5) << "좌표 전송 완료: " << json_str << std::endl;
            }

            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
        }
    }

    void send_pose_to_server23(float x3, float y3) {
        json data;
        data["x"] = x3;
        data["y"] = y3;
        std::string json_str = data.dump();

        CURL* curl = curl_easy_init();
        if (curl) {
            const std::string url = "http://192.168.202.66:5000/report_position3";  // ← 여기에 Flask 서버 IP

            struct curl_slist* headers = nullptr;
            headers = curl_slist_append(headers, "Content-Type: application/json");

            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_str.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "curl failed: " << curl_easy_strerror(res) << std::endl;
            } else {
                //std::cout << "좌표 전송 완료: " << json_str << std::endl;
                std::cout << std::fixed << std::setprecision(5) << "좌표 전송 완료23: " << json_str << std::endl;
            }

            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
        }
    }

    void send_pose_to_server42(float x, float y) {
        json data;
        data["x"] = x;
        data["y"] = y;
        std::string json_str = data.dump();

        CURL* curl = curl_easy_init();
        if (curl) {
            const std::string url = "http://192.168.202.66:5000/report_position";  // ← 여기에 Flask 서버 IP

            struct curl_slist* headers = nullptr;
            headers = curl_slist_append(headers, "Content-Type: application/json");

            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_str.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "curl failed: " << curl_easy_strerror(res) << std::endl;
            } else {
                //std::cout << "좌표 전송 완료: " << json_str << std::endl;
                std::cout << std::fixed << std::setprecision(5) << "좌표 전송 완료: " << json_str << std::endl;
            }

            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
        }
    }

    void send_pose_to_server84(float x, float y) {
        json data;
        data["x"] = x;
        data["y"] = y;
        std::string json_str = data.dump();

        CURL* curl = curl_easy_init();
        if (curl) {
            const std::string url = "http://192.168.202.66:5000/report_position";  // ← 여기에 Flask 서버 IP

            struct curl_slist* headers = nullptr;
            headers = curl_slist_append(headers, "Content-Type: application/json");

            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_str.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "curl failed: " << curl_easy_strerror(res) << std::endl;
            } else {
                //std::cout << "좌표 전송 완료: " << json_str << std::endl;
                std::cout << std::fixed << std::setprecision(5) << "좌표 전송 완료: " << json_str << std::endl;
            }

            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
        }
    }

    void send_pose_to_server101(float x1, float y1) {
        json data;
        data["x"] = x1;
        data["y"] = y1;
        std::string json_str = data.dump();

        CURL* curl = curl_easy_init();
        if (curl) {
            const std::string url = "http://192.168.202.66:5000/report_position1";  // ← 여기에 Flask 서버 IP

            struct curl_slist* headers = nullptr;
            headers = curl_slist_append(headers, "Content-Type: application/json");

            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_str.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "curl failed: " << curl_easy_strerror(res) << std::endl;
            } else {
                //std::cout << "좌표 전송 완료: " << json_str << std::endl;
                std::cout << std::fixed << std::setprecision(5) << "좌표 전송 완료: " << json_str << std::endl;
            }

            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
        }
    }

    void send_pose_to_server156(float x2, float y2) {
        json data;
        data["x"] = x2;
        data["y"] = y2;
        std::string json_str = data.dump();

        CURL* curl = curl_easy_init();
        if (curl) {
            const std::string url = "http://192.168.202.66:5000/report_position2";  // ← 여기에 Flask 서버 IP

            struct curl_slist* headers = nullptr;
            headers = curl_slist_append(headers, "Content-Type: application/json");

            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_str.c_str());
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                std::cerr << "curl failed: " << curl_easy_strerror(res) << std::endl;
            } else {
                //std::cout << "좌표 전송 완료: " << json_str << std::endl;
                std::cout << std::fixed << std::setprecision(5) << "좌표 전송 완료156: " << json_str << std::endl;
            }

            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
        }
    }
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void collectSamples_Pixel(std::vector<double>& x_samples, std::vector<double>& y_samples, 
                            double x, double y, bool& is_fixed, double& x_fixed, double& y_fixed) {
        if (!is_fixed) {
            if (x_samples.size() < sample_count) {
                x_samples.push_back(x);
                y_samples.push_back(y);
            }

            if (x_samples.size() == sample_count) {
                x_fixed = std::accumulate(x_samples.begin(), x_samples.end(), 0.0) / sample_count;
                y_fixed = std::accumulate(y_samples.begin(), y_samples.end(), 0.0) / sample_count;

                ROS_INFO("Fixed Pixel Position set: [x: %f, y: %f]", x_fixed, y_fixed);
                is_fixed = true;

                // 샘플 벡터를 비워서 메모리를 절약할 수 있습니다.
                x_samples.clear();
                y_samples.clear();
            }
        }
    }

    void collectSamples(std::vector<double>& x_samples, std::vector<double>& y_samples, std::vector<double>& z_samples,
                        double x, double y, double z, bool& is_fixed, double& x_fixed, double& y_fixed, double& z_fixed) {
        if (x_samples.size() < sample_count) {
            x_samples.push_back(x);
            y_samples.push_back(y);
            z_samples.push_back(z);
        }

        if (x_samples.size() == sample_count && !is_fixed) {
            x_fixed = std::accumulate(x_samples.begin(), x_samples.end(), 0.0) / sample_count;
            y_fixed = std::accumulate(y_samples.begin(), y_samples.end(), 0.0) / sample_count;
            z_fixed = std::accumulate(z_samples.begin(), z_samples.end(), 0.0) / sample_count;

            ROS_INFO("Fixed Position set: [x: %f, y: %f, z: %f]", x_fixed, y_fixed, z_fixed);
            is_fixed = true;
        }
    }

    void publishMarkers() {
        visualization_msgs::MarkerArray marker_array;

        addMarker(marker_array, 23, marker23_x_fixed, marker23_y_fixed, marker23_z_fixed);
        addMarker(marker_array, 42, marker42_x_fixed, marker42_y_fixed, marker42_z_fixed);
        addMarker(marker_array, 84, marker84_x_fixed, marker84_y_fixed, marker84_z_fixed);
        //addMarker(marker_array, 101, marker101_x_fixed, marker101_y_fixed, marker101_z_fixed);

        marker_pub_.publish(marker_array);
    }

    void addMarker(visualization_msgs::MarkerArray& marker_array, int markerId, double x, double y, double z) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera";
        marker.header.stamp = ros::Time::now();
        marker.ns = "aruco_markers";
        marker.id = markerId;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;  // 구의 크기
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;  // 투명도
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;  // 파란색

        marker_array.markers.push_back(marker);

        // 텍스트 마커 추가
        visualization_msgs::Marker text_marker = marker;
        text_marker.id = markerId + 1000;  // 텍스트 마커의 ID는 고유해야 함
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.text = "Marker " + std::to_string(markerId);
        text_marker.scale.z = 0.05;  // 텍스트 크기
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;  // 투명도 유지

        text_marker.pose.position.z += 0.1;  // 텍스트가 구 위에 위치하도록

        marker_array.markers.push_back(text_marker);
    }

    void publishGlobalFrame() {
        // 1. 마커들로 만든 평면의 법선 벡터 계산
        tf2::Vector3 p1(marker23_x_fixed, marker23_y_fixed, marker23_z_fixed);
        tf2::Vector3 p2(marker42_x_fixed, marker42_y_fixed, marker42_z_fixed);
        tf2::Vector3 p3(marker101_x_fixed, marker101_y_fixed, marker101_z_fixed);

        // 벡터 p1p2와 p1p3의 외적을 통해 법선 벡터 계산
        tf2::Vector3 v1 = p2 - p1;
        tf2::Vector3 v2 = p3 - p1;
        tf2::Vector3 normal = v1.cross(v2);  // 평면의 법선 벡터
        normal.normalize();  // 정규화

        // 2. 법선 벡터와 카메라 z축의 내적을 확인하여 방향 조정
        tf2::Vector3 camera_z(0.0, 0.0, 1.0);  // 카메라 프레임의 z축
        if (normal.dot(camera_z) > 0) {
            normal = -normal;  // 법선 벡터의 방향을 반대로 바꿈
        }

        // 3. Global 프레임의 중심점 계산
        double center_x = marker23_x_fixed; // 원래 101
        double center_y = marker23_y_fixed;
        double center_z = marker23_z_fixed;

        // 4. x축과 y축 계산
        tf2::Vector3 up_vector(0.0, 1.0, 0.0);  // 임의의 y축 방향
        tf2::Vector3 x_axis_global = up_vector.cross(normal);  // z축(법선 벡터)에 수직인 x축 계산
        x_axis_global.normalize();  // 정규화
        tf2::Vector3 y_axis_global = normal.cross(x_axis_global);  // 다시 y축을 계산하여 정규화
        y_axis_global.normalize();

        // 5. 회전 행렬 생성
        tf2::Matrix3x3 rotation_matrix(
            x_axis_global.getX(), y_axis_global.getX(), normal.getX(),
            x_axis_global.getY(), y_axis_global.getY(), normal.getY(),
            x_axis_global.getZ(), y_axis_global.getZ(), normal.getZ()
        );

        // 6. 회전 행렬에서 쿼터니언 계산
        tf2::Quaternion q;
        rotation_matrix.getRotation(q);

        // 7. TF 메시지 생성 및 publish
        global_transform.header.stamp = ros::Time::now();
        global_transform.header.frame_id = "camera";  // Camera frame을 기준으로 함
        global_transform.child_frame_id = "Global";  // 새로운 Global frame
        
        global_transform.transform.translation.x = center_x;
        global_transform.transform.translation.y = center_y;
        global_transform.transform.translation.z = center_z;
        global_transform.transform.rotation.x = q.x();
        global_transform.transform.rotation.y = q.y();
        global_transform.transform.rotation.z = q.z();
        global_transform.transform.rotation.w = q.w();

        br_.sendTransform(global_transform);
    }

    void projectMarkersOntoGlobalFrame() {
        // 1. Global 프레임 평면의 기준점 (마커 101의 위치)
        tf2::Vector3 marker101_pos(marker101_x_fixed, marker101_y_fixed, marker101_z_fixed);

        // 2. Global 프레임 평면의 법선 벡터 (이미 계산된 normal 벡터 사용)
        tf2::Vector3 p1(marker23_x_fixed, marker23_y_fixed, marker23_z_fixed);
        tf2::Vector3 p2(marker42_x_fixed, marker42_y_fixed, marker42_z_fixed);
        tf2::Vector3 p3(marker101_x_fixed, marker101_y_fixed, marker101_z_fixed);
        tf2::Vector3 v1 = p2 - p1;
        tf2::Vector3 v2 = p3 - p1;
        tf2::Vector3 normal = v1.cross(v2);
        normal.normalize();

        // Function to project a marker onto the global frame
        auto project_marker = [&](double& x, double& y, double& z) {
            tf2::Vector3 marker_pos(x, y, z);
            tf2::Vector3 vector_to_plane = marker_pos - marker101_pos;
            double distance_to_plane = vector_to_plane.dot(normal);
            tf2::Vector3 projected_pos = marker_pos - distance_to_plane * normal;
            x = projected_pos.getX();
            y = projected_pos.getY();
            z = projected_pos.getZ();
        };

        // Project each marker onto the global frame plane
        project_marker(marker23_x_fixed, marker23_y_fixed, marker23_z_fixed);
        project_marker(marker42_x_fixed, marker42_y_fixed, marker42_z_fixed);
        project_marker(marker84_x_fixed, marker84_y_fixed, marker84_z_fixed);
        project_marker(marker101_x_fixed, marker101_y_fixed, marker101_z_fixed);
        project_marker(marker156_x_fixed, marker156_y_fixed, marker156_z_fixed);  // 로봇 위치도 정사영
    }

    void visualizeRobotInGlobalFrame() {
        visualization_msgs::MarkerArray marker_array;

        // 로봇 위치를 시각화 (초록색 구)
        addRobotMarker(marker_array, 101, marker101_x_fixed, marker101_y_fixed, marker101_z_fixed);
        addRobotMarker(marker_array, 156, marker156_x_fixed, marker156_y_fixed, marker156_z_fixed);

        marker_pub_.publish(marker_array);
    }

    void addRobotMarker(visualization_msgs::MarkerArray& marker_array, int markerId, double x, double y, double z) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera";  // Global frame 기준
        marker.header.stamp = ros::Time::now();
        marker.ns = "aruco_markers";
        marker.id = markerId;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;  // 구의 크기
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;  // 투명도
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;  // 초록색

        marker_array.markers.push_back(marker);

        // 텍스트 마커 추가
        visualization_msgs::Marker text_marker = marker;
        text_marker.id = markerId + 1000;  // 텍스트 마커의 ID는 고유해야 함
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.text = "Robot Position";
        text_marker.scale.z = 0.1;  // 텍스트 크기
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.color.a = 1.0;  // 투명도 유지

        text_marker.pose.position.z += 0.15;  // 텍스트가 구 위에 위치하도록

        marker_array.markers.push_back(text_marker);
    }

    void logMarkersInGlobalFrame() {
        // 1. Global 프레임의 변환 행렬과 회전 설정
        tf2::Vector3 global_origin(marker101_x_fixed, marker101_y_fixed, marker101_z_fixed);
        
        tf2::Quaternion q;
        q.setX(global_transform.transform.rotation.x);        

        q.setY(global_transform.transform.rotation.y);
        q.setZ(global_transform.transform.rotation.z);
        q.setW(global_transform.transform.rotation.w);
        
        tf2::Matrix3x3 rotation_matrix(q);

        // 2. 각 마커의 좌표를 Global 프레임으로 변환
        auto transform_to_global = [&](double x, double y, double z) {
            tf2::Vector3 point(x, y, z);
            tf2::Vector3 point_in_global = rotation_matrix * (point - global_origin);
            return point_in_global;
        };

        tf2::Vector3 marker23_global = transform_to_global(marker23_x_fixed, marker23_y_fixed, marker23_z_fixed);
        tf2::Vector3 marker42_global = transform_to_global(marker42_x_fixed, marker42_y_fixed, marker42_z_fixed);
        tf2::Vector3 marker84_global = transform_to_global(marker84_x_fixed, marker84_y_fixed, marker84_z_fixed);
        tf2::Vector3 marker101_global = transform_to_global(marker101_x_fixed, marker101_y_fixed, marker101_z_fixed);
        tf2::Vector3 marker156_global = transform_to_global(marker156_x_fixed, marker156_y_fixed, marker156_z_fixed);  // 로봇 위치도 변환

        // 3. 변환된 좌표를 ROS_INFO로 출력
        ROS_INFO("Marker 23 in Global Frame: [x: %f, y: %f, z: %f]", marker23_global.getX(), marker23_global.getY(), marker23_global.getZ());
        //ROS_INFO("Marker 42 in Global Frame: [x: %f, y: %f, z: %f]", marker42_global.getX(), marker42_global.getY(), marker42_global.getZ());
        //ROS_INFO("Marker 84 in Global Frame: [x: %f, y: %f, z: %f]", marker84_global.getX(), marker84_global.getY(), marker84_global.getZ());
        //ROS_INFO("Marker 101 in Global Frame: [x: %f, y: %f, z: %f]", marker101_global.getX(), marker101_global.getY(), marker101_global.getZ());
        //ROS_INFO("Robot (Marker 156) in Global Frame: [x: %f, y: %f, z: %f]", marker156_global.getX(), marker156_global.getY(), marker156_global.getZ());
        // float x = marker156_global.getX();
        // float y = marker156_global.getY();
        // send_pose_to_server(x,y);
    }

    // 1. human_Pixel2Global 함수: 픽셀 좌표에서 Global 좌표로 변환
    void human_Pixel2Global() {
        // Human1의 픽셀 좌표 (예: (800, 180))
        cv::Point2f human_pixel(400, 300); // 이거 Pixel Frame을 marker 101가 (0,0)일 때를 기준으로 나타낸 것임.
        // cv::Point2f human_pixel(1000, 500);
        // cv::Point2f human_pixel(1000, 500);

        // 1. 호모그래피 행렬을 이용하여 픽셀 좌표를 Global 좌표로 변환
        std::vector<cv::Point2f> pixel_points = {human_pixel};
        std::vector<cv::Point2f> global_points;

        if (is_transformed) {
            cv::perspectiveTransform(pixel_points, global_points, transformation_matrix);

            // 2. 변환된 Global 좌표를 출력
            ROS_INFO("Human1 in Global Coordinates: [x: %f, y: %f]", global_points[0].x, global_points[0].y);

            // 3. 변환된 좌표를 addHumanMarker 함수로 넘겨서 시각화
            addHumanMarker(global_points[0].x, global_points[0].y, 0.0);  // z는 0으로 설정
        } else {
            ROS_WARN("Homography matrix is not calculated yet!");
        }
    }

    // 2. addHumanMarker 함수: 변환된 Human1 좌표를 RViz에서 시각화
    void addHumanMarker(double x, double y, double z) {
        visualization_msgs::MarkerArray marker_array;

        // Human1의 마커 생성
        visualization_msgs::Marker human_marker;
        human_marker.header.frame_id = "Global";  // Global 프레임 기준
        human_marker.header.stamp = ros::Time::now();
        human_marker.ns = "human_markers";
        human_marker.id = 1;  // Human1 마커 ID
        human_marker.type = visualization_msgs::Marker::SPHERE;
        human_marker.action = visualization_msgs::Marker::ADD;
        human_marker.pose.position.x = x;
        human_marker.pose.position.y = y;
        human_marker.pose.position.z = z;  // z값은 고정
        human_marker.pose.orientation.x = 0.0;
        human_marker.pose.orientation.y = 0.0;
        human_marker.pose.orientation.z = 0.0;
        human_marker.pose.orientation.w = 1.0;
        human_marker.scale.x = 0.2;  // 구의 크기           
        human_marker.scale.y = 0.2;
        human_marker.scale.z = 0.2;
        human_marker.color.a = 1.0;  // 투명도
        human_marker.color.r = 1.0;
        human_marker.color.g = 0.0;
        human_marker.color.b = 0.0;  // 파란색 (사람 마커 색)

        // Human1 마커 추가
        marker_array.markers.push_back(human_marker);

        // 텍스트 마커 추가 (Human1)
        visualization_msgs::Marker text_marker = human_marker;
        text_marker.id = 1001;  // 텍스트 마커 ID
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.text = "Human1";
        text_marker.scale.z = 0.1;  // 텍스트 크기
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;  // 흰색 텍스트
        text_marker.color.a = 1.0;  // 투명도 유지

        text_marker.pose.position.z += 0.3;  // 텍스트가 구 위에 위치하도록

        // Human1 텍스트 마커 추가
        marker_array.markers.push_back(text_marker);

        // 마커 퍼블리시
        marker_pub_.publish(marker_array);
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_pose_node");
    ArucoPoseNode aruco;

    // 메인 스핀 루프
    ros::Rate loop_rate(20);  // 20Hz로 루프를 실행

    while (ros::ok()) {
        // ROS 콜백 함수 호출 (imageCallback 등)
        ros::spinOnce();

        // 매 루프마다 publishMapTransform을 호출하여 기존 타이머 기능을 대체
        aruco.publishMapTransform();

        // 20Hz 루프를 유지
        loop_rate.sleep();
    }

    return 0;
}

