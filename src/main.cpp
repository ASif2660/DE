
#include "sl/Camera.hpp"
#include "opencv2/aruco.hpp"
/*
 * Program developed by CANM
 * Contact person: Ergun Yavuz
 * email:  Ergun.Yavuz@ServiceXpert.de and Mohammed.Chand@ServiceXpert.de
 *
 */


/* TODO: include all the headers of the SDK
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
    init_params.depth_mode = sl::DEPTH_MODE_ULTRA;
    init_params.coordinate_units = sl::UNIT_MILLIMETER;



    //initialize OpenCV related parameters
    cv::Mat marker_Image; //create an aruco marker
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); // take some random aruco image
    cv::aruco::drawMarker(dictionary, 23, 200, marker_Image, 1);


    // Define the Camera Object and related data types
    sl::Camera zed;
    sl::ERROR_CODE zed_camera_error = zed.open(init_params);
    sl::Mat Image;
    sl::Mat depth_map;


    if(zed_camera_error != sl::SUCCESS) {// Quit if an error occurred
        std::cout << zed_camera_error << std::endl;
        zed.close();
        return 1;
    }



    sl::Resolution resolution = zed.getResolution();




    if(zed.grab() == sl::SUCCESS) {

    std::cout << resolution.width << " is width and height is " << resolution.height << std::endl;



    }








    return 0;
}
