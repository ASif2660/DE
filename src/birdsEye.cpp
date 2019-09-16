//
// Created by asif on 16.09.19.
//

#include "birdsEye.h"


void birdsEye::trackbarFunction() {



    int alpha_ = 20, beta_ = 90, gamma_ = 90;
    int f_x = _cal_params.fx, dist_ = 500;
    int f_y = _cal_params.fy;


    cv::namedWindow("BirdsEye", 1);

  //  cv::createTrackbar("Alpha", "BirdsEye", &alpha_, 180);
  /*  cv::createTrackbar("Beta", "BirdsEye", &beta_, 180);
    cv::createTrackbar("Gamma", "Result", &gamma_, 180);
    cv::createTrackbar("f_x", "Result", &f_x, 2000);
    cv::createTrackbar("f_y","Result", &f_y, 2000);
    cv::createTrackbar("Distance", "Result", &dist_, 2000);
*/


    double focalLength_x, focalLength_y, dist, alpha, beta, gamma;

    alpha =((double)alpha_ -90) * PI/180;
    beta =((double)beta_ -90) * PI/180;
    gamma =((double)gamma_ -90) * PI/180;
    focalLength_x = (double)f_x;
    focalLength_y = (double)f_y;
    dist = (double)dist_;


    cv::Size image_size = _source.size();

    double w = (double)image_size.width, h = (double)image_size.height;


    cv::Mat A1 = (cv::Mat_ <float>(4,3) << 1, 0, -w/2,
            0, 1, -h/2,
            0, 0, 0,
            0, 0, 1 );

    cv::Mat RX = (cv::Mat_<float>(4, 4) <<
                                1, 0, 0, 0,
            0, cos(alpha), -sin(alpha), 0,
            0, sin(alpha), cos(alpha), 0,
            0, 0, 0, 1 );

    cv::Mat RY = (cv::Mat_<float>(4, 4) <<
                                cos(beta), 0, -sin(beta), 0,
            0, 1, 0, 0,
            sin(beta), 0, cos(beta), 0,
            0, 0, 0, 1	);

    cv::Mat RZ = (cv::Mat_<float>(4, 4) <<
                                cos(gamma), -sin(gamma), 0, 0,
            sin(gamma), cos(gamma), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1	);


    // R - rotation matrix
    cv::Mat R = RX * RY * RZ;

    // T - translation matrix
    cv::Mat T = (cv::Mat_<float>(4, 4) <<
                               1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, dist,
            0, 0, 0, 1);

    // K - intrinsic matrix
    cv::Mat K = (cv::Mat_<float>(3, 4) <<
                               focalLength_x, 0, w/2, 0,
            0, focalLength_y, h/2, 0,
            0, 0, 1, 0
    );


    cv::Mat transformationMat = K * (T * (R * A1));

    char str[200];

    sprintf(str,"WARPED IMAGE, ALPHA = 25");

    warpPerspective(_source, _destination, transformationMat, image_size, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);

    putText(_destination, str, cv::Point2f(100,100), cv::FONT_HERSHEY_PLAIN, 2,  cv::Scalar(0,255,0,255));


    imshow("BirdsEye", _destination);

    cv::waitKey(10);
}








double birdsEye::scalingFactor(std::vector< std::vector<cv::Point2f> >& aruco_corners, sl::Resolution aruco_resolution  ) {



}







