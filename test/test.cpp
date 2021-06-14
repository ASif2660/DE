
#include "sl/Camera.hpp"
#include "opencv2/aruco.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "zed2cv.h"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <birdsEye.h>



//TODO: Create Test Cases with catch2 or Gtest


#include <typeinfo>



int main(int argc, char* argv[] ) {

    //initialize the Configuration parameters of the ZED Camera

    sl::InitParameters init_params;
    //  init_params.enable_right_side_measure = true;
    init_params.depth_mode = sl::DEPTH_MODE_ULTRA;
    init_params.coordinate_units = sl::UNIT_MILLIMETER;


    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);




    // Define the Camera Object and related data types
    sl::Camera zed;
    sl::ERROR_CODE zed_camera_error = zed.open(init_params);
    sl::Mat depth_image(zed.getResolution(), sl::MAT_TYPE_8U_C1); // this is going to be an RGBA image
    sl::Mat image_full(zed.getResolution(), sl::MAT_TYPE_8U_C4);
    sl::Mat depth_calculate(zed.getResolution(), sl::MAT_TYPE_32F_C1);
    float depth_center_pixel = 0;
    cv::int16_t x = 0;
    cv::int16_t y = 0;



    // BirdsEye Params the class object
    cv::Mat frame;
    cv::Mat img_denoise;
    cv::Mat img_edges;
    cv::Mat img_mask;
    cv::Mat img_lines;
    std::vector<cv::Vec4i> lines;
    std::vector<std::vector<cv::Vec4i> > left_right_lines;
    std::vector<cv::Point> lane;
    std::string turn;
    int flag_plot = -1;
    int iterator = 0;
    float fx_camera =  zed.getCameraInformation().calibration_parameters.left_cam.fx;
    float fy_camera =  zed.getCameraInformation().calibration_parameters.left_cam.fy;

    cv::Mat final_image;





    while (1) {
        if (zed.grab() == sl::SUCCESS) { //1280x720



            zed.retrieveImage(image_full, sl::VIEW_LEFT);


            cv::Mat cv_RGB_image = slMat2cvMat(image_full);

            cvtColor(cv_RGB_image, cv_RGB_image, CV_RGBA2RGB);


            birdsEye surroundEye(cv_RGB_image, fx_camera, fy_camera);

            img_denoise = surroundEye.deNoise(cv_RGB_image);

            img_edges = surroundEye.edgeDetector(img_denoise);

            img_mask = surroundEye.mask(img_edges);

            lines = surroundEye.houghLines(img_mask);




            if (!lines.empty()) {
                // Separate lines into left and right lines
                left_right_lines = surroundEye.lineSeparation(lines, img_edges);

                // Apply regression to obtain only one line for each side of the lane
                lane = surroundEye.regression(left_right_lines, cv_RGB_image);

                // Predict the turn by determining the vanishing point of the the lines
                turn = surroundEye.predictTurn();

                // Plot lane detection
                cv::Mat with_lane_image = surroundEye.plotLane(cv_RGB_image, lane, turn);

                cv::Mat warpedImage = surroundEye.birdsEyeFunction(with_lane_image);


                cv::imshow("totalImage", warpedImage);

                cv::waitKey(50);

              //  iterator += 1;

            }

         //   cv::imshow("Final_image", final_image);

        //    cv::imshow("Test", cv_RGB_image);

            //cv::waitKey(100);

        } else sl::sleep_ms(10);

    }
        cv::destroyAllWindows();

        return 0;


}

