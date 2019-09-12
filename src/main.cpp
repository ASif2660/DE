
#include "sl/Camera.hpp"
#include "opencv2/aruco.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "zed2cv.h"
#include <iostream>
/*
 * Program developed by CANM
 * Contact person: Ergun Yavuz
 * email:  Ergun.Yavuz@ServiceXpert.de and Mohammed.Chand@ServiceXpert.de
 *
 */


/* TODO: DONE
 * TODO: include all the headers of ARUCO
 * TODO: locate the ARUCO Marker with a test program
 * TODO: locate the average depth of a bounded region using the SDK functions
 * TODO: Integrate both the functions
 * TODO: cv::Mat to sl::Mat conversion
 * TODO: Tracking ?
 *
 * /

 */


int main(int argc, char* argv[] ) {

    //initialize the Configuration parameters of the ZED Camera

    sl::InitParameters init_params;
//    init_params.enable_right_side_measure = true;
    init_params.depth_mode = sl::DEPTH_MODE_ULTRA;
    init_params.coordinate_units = sl::UNIT_MILLIMETER;



    //initialize OpenCV related parameters
    cv::Mat marker_Image; //create an aruco marker
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); // take some random aruco image
    cv::aruco::drawMarker(dictionary, 23, 200, marker_Image, 1);


    // Define the Camera Object and related data types
    sl::Camera zed;
    sl::ERROR_CODE zed_camera_error = zed.open(init_params);
    sl::Mat depth_image(zed.getResolution(), sl::MAT_TYPE_32F_C1); // this is going to be an RGBA image
    sl::Mat image_full(zed.getResolution(), sl::MAT_TYPE_32F_C4);



for (int i = 0; i < 10; i++ ) {

    if (zed.grab() == sl::SUCCESS) {// Quit if an error occurred
        //std::cout << zed_camera_error << std::endl;
        // std::cout <<  zed.getResolution << std::endl;/ this is 1280 and 720



        zed.retrieveMeasure(depth_image, sl::MEASURE_DEPTH);
  /*      std::cout << "Resolution of depth image " << depth_image.getResolution().height << depth_image.getResolution().width << std::endl;

        zed.retrieveMeasure(image_full,sl::MEASURE_XYZBGRA_RIGHT);
        std::cout << "Resolution of right image is " <<  image_full.getResolution().height << depth_image.getResolution().width << std::endl;
  */

        // Convert ZED Camera Mat objects to CV::Mat Objects
        cv::Mat cv_depth_image = slMat2cvMat(depth_image);

        float depth = cv_depth_image.at<float>(cv_depth_image.rows / 2, cv_depth_image.cols / 2);


        std::cout << " The depth of centermost pixel is " << depth << std::endl;

 //       std::string value = "Depth value " + std::to_string(depth);


          cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);

          cv::imshow( "value ",cv_depth_image );

          cv::waitKey(0);

          cv::destroyWindow("Depth");




    } else sl::sleep_ms(10);

}



    return 0;
}
