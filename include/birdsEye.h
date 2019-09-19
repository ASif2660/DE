//
// Created by asif on 16.09.19.
//

#ifndef DE_BIRDSEYE_H
#define DE_BIRDSEYE_H

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <vector>
#include <string>
#include <opencv2/highgui/highgui.hpp>

#define PI 3.1415926

class birdsEye {

private:

    cv::Mat _source;
    cv::Mat _destination;
    cv::Matx33d camera_mat;
    const size_t ARUCO_MARKER = 180; // we can get the scaling factor
   // sl::CameraParameters _cal_params;
    double img_size;
    double img_center;
    bool left_flag = false;  // Tells us if there's left boundary of lane detected
    bool right_flag = false;  // Tells us if there's right boundary of lane detected
    cv::Point right_b;  // Members of both line equations of the lane boundaries:
    double right_m;  // y = m*x + b
    cv::Point left_b;  //
    double left_m;
    double m_camera_fx;
    double m_camera_fy;





public:

    birdsEye(cv::Mat& source_image, float camera_fx, float camera_fy): _source(source_image){

        m_camera_fx = camera_fx;
        m_camera_fy = camera_fy;


    }

   ~birdsEye(){}

    cv::Mat birdsEyeFunction(cv::Mat plotted_image );
    double scalingFactor(std::vector< std::vector<cv::Point2f> >& aruco_corners, sl::Resolution aruco_resolution  );

    cv::Mat deNoise(cv::Mat inputImage);  // Apply Gaussian blurring to the input Image
    cv::Mat edgeDetector(cv::Mat img_noise);  // Filter the image to obtain only edges
    cv::Mat mask(cv::Mat img_edges);  // Mask the edges image to only care about ROI

    std::vector<cv::Vec4i> houghLines(cv::Mat img_mask);  // Detect Hough lines in masked edges image
    std::vector<std::vector<cv::Vec4i> > lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges);  // Sprt detected lines by their slope into right and left lines
    std::vector<cv::Point> regression(std::vector<std::vector<cv::Vec4i> > left_right_lines, cv::Mat inputImage);  // Get only one line for each side of the lane
    std::string predictTurn();  // Determine if the lane is turning or not by calculating the position of the vanishing point
    cv::Mat plotLane(cv::Mat inputImage, std::vector<cv::Point> lane, std::string turn);  // Plot the resultant lane and turn prediction in the frame.




};


















#endif //DE_BIRDSEYE_H
