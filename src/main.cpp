
#include "sl/Camera.hpp"
#include "opencv2/aruco.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "zed2cv.h"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>





#include <typeinfo>

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
    init_params.enable_right_side_measure = true;
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
    float depth_center_pixel =0;
    cv::int16_t x = 0;
    cv::int16_t y = 0;




for (int k = 0; k < 100; k++ ) {

    if (zed.grab() == sl::SUCCESS) { //1280x720



        zed.retrieveImage(depth_image, sl::VIEW_DEPTH_RIGHT);
        zed.retrieveImage(image_full, sl::VIEW_RIGHT);
        zed.retrieveMeasure(depth_calculate, sl::MEASURE_DEPTH);



        // Convert ZED Camera Mat objects to CV::Mat Objects
        cv::Mat cv_depth_image = slMat2cvMat(depth_image);
        cv::Mat cv_RGB_image = slMat2cvMat(image_full);




        cvtColor(cv_depth_image, cv_depth_image, CV_8UC1);
        cvtColor(cv_RGB_image, cv_RGB_image,CV_RGBA2RGB);


  /*

        float depth = cv_depth_image.at<float>(cv_depth_image.rows / 2, cv_depth_image.cols / 2);


        std::cout << " The depth of centermost pixel is " << depth << std::endl;

        std::string value = "Depth value " + std::to_string(depth);

*/
         cv::Mat cv_RGB_image_copy;
         cv_RGB_image.copyTo(cv_RGB_image_copy);
         cv::aruco::detectMarkers(cv_RGB_image, dictionary, markerCorners, markerIds);


         if( markerIds.size() > 0) {


             //       std::cout << " Found the marker " << std::endl;



             cv::aruco::drawDetectedMarkers(cv_RGB_image_copy, markerCorners, markerIds);




             for (std::vector<int>::size_type i = 0; i < markerCorners.size(); i++) {

                 for (std::vector<int>::size_type j = 0; j < markerCorners[i].size(); j++) {

//                    std::cout << "Four Corners are " << markerCorners[i][j].x << typeid(markerCorners[i][j].x).name() << std::endl;


                     x += markerCorners[i][j].x;

                     y += markerCorners[i][j].y;


                     std::cout << " Inner for loop " << "x  and y" << x << " " << y << std::endl;

                 }

             }

             x = int(x) / 4;
             y = int(y) / 4;




             exit(0);
         }


             // we are basically saying we have enough ids inside the vector of the markerIds

            std::cout << "Thje raw value is " << x  <<  "and " << y << std::endl;


            if (x < 1280 && y < 720 ) {

                depth_calculate.getValue(y,x,&depth_center_pixel);

                std::cout <<  "The depth is " << depth_center_pixel << std::endl;

         }


        cv::namedWindow("ARUCO_RGB", CV_WINDOW_AUTOSIZE);

        cv::imshow( "ARUCO_RGB ", cv_depth_image);

        cv::waitKey(200);


     } else sl::sleep_ms(10);

    } //grabber loop end


    cv::destroyAllWindows();

    return 0;
}
