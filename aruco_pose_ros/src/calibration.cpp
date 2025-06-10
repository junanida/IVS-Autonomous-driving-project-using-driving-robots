#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    // 인자 체크: 올바른 사용법을 안내
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <checkboard_images_directory>" << std::endl;
        return -1;
    }

    // 체커보드 이미지가 있는 디렉터리 경로를 저장
    std::string dir = argv[1];
    std::vector<std::string> image_files;
    // 디렉터리 내의 모든 .png 파일을 찾음
    cv::glob(dir + "/*.png", image_files, false);

    // 이미지 파일이 없을 경우 에러 메시지 출력
    if (image_files.empty()) {
        std::cerr << "No images found in the specified directory!" << std::endl;
        return -1;
    }

    // 2D 이미지 포인트와 3D 객체 포인트를 저장할 벡터
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;
    // 체커보드의 3D 포인트를 저장할 벡터
    std::vector<cv::Point3f> obj;

    // 체커보드의 크기와 각 사각형의 한 변의 길이 (단위: 미터)
    int board_width = 9;
    int board_height = 6;
    float square_size = 0.025f;

    // 체커보드의 각 코너에 대한 3D 좌표를 초기화
    for (int i = 0; i < board_height; ++i) {
        for (int j = 0; j < board_width; ++j) {
            obj.push_back(cv::Point3f(j * square_size, i * square_size, 0));
        }
    }

    // 각 이미지 파일에 대해 처리
    for (const auto& file : image_files) {
        // 이미지를 그레이스케일로 읽어들임
        cv::Mat image = cv::imread(file, cv::IMREAD_GRAYSCALE);
        if (image.empty()) {
            std::cerr << "Failed to load image: " << file << std::endl;
            continue;
        }

        // 체커보드 코너를 저장할 벡터
        std::vector<cv::Point2f> corners;
        // 체커보드 코너를 검출
        bool found = cv::findChessboardCorners(image, cv::Size(board_width, board_height), corners,
                                               cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if (found) {
            // 코너 위치를 더 정밀하게 계산
            cv::cornerSubPix(image, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));
            image_points.push_back(corners);
            object_points.push_back(obj);

            // 검출된 코너를 이미지에 그려서 시각화
            cv::drawChessboardCorners(image, cv::Size(board_width, board_height), corners, found);
            cv::imshow("Chessboard detection", image);
            cv::waitKey(100);
        }
    }

    // 모든 이미지 창을 닫음
    cv::destroyAllWindows();

    // 유효한 이미지가 충분하지 않으면 에러 메시지 출력
    if (image_points.size() < 1) {
        std::cerr << "Not enough valid images for calibration!" << std::endl;
        return -1;
    }

    // 카메라 매트릭스와 왜곡 계수를 저장할 행렬
    cv::Mat camera_matrix, dist_coeffs;
    // 회전 벡터와 변환 벡터를 저장할 벡터
    std::vector<cv::Mat> rvecs, tvecs;
    // 카메라 보정 수행
    cv::calibrateCamera(object_points, image_points, cv::Size(image.cols, image.rows), camera_matrix, dist_coeffs, rvecs, tvecs);

    // 결과 출력
    std::cout << "Camera Matrix: " << std::endl << camera_matrix << std::endl;
    std::cout << "Distortion Coefficients: " << std::endl << dist_coeffs << std::endl;

    return 0;
}
