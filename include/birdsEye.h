//
// Created by asif on 16.09.19.
//

#ifndef DE_BIRDSEYE_H
#define DE_BIRDSEYE_H

#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <vector>
#define PI 3.1415926

class birdsEye {

private:

    cv::Mat _source;
    cv::Mat _destination;
    cv::Matx33d camera_mat;
    const size_t ARUCO_MARKER = 180; // we can get the scaling factor
    sl::CameraParameters _cal_params;




public:

    birdsEye(cv::Mat& source_image, cv::Matx33d& camera_matrix, sl::CameraParameters calibrate): _source(source_image), _cal_params(calibrate){



    }

   ~birdsEye(){}

    void trackbarFunction();



    double scalingFactor(std::vector< std::vector<cv::Point2f> >& aruco_corners, sl::Resolution aruco_resolution  );

};



#endif //DE_BIRDSEYE_H
