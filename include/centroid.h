//
// Created by asif on 17.09.19.
//

#ifndef DE_CENTROID_H
#define DE_CENTROID_H
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>




inline float centroid_t( float x1, float x2, float x3 ){

    return x1 + x2 + x3 / 3.0;

}





float centroid ( std::vector<std::vector<cv::Point2f> > some_corners) {


    float  x1 = some_corners[0][0].x;
    float  y1 = some_corners[0][0].y;
    float  x2 = some_corners[0][1].x;
    float  y2 = some_corners[0][1].y;
    float  x3 = some_corners[0][2].x;
    float  y3 = some_corners[0][2].y;
    float  x4 = some_corners[0][3].x;
    float  y4 = some_corners[0][3].y;


    // center of triangle 1 2 3

    float centroid_t1_x = centroid_t(x1, x2, x3);
    float centroid_t1_y = centroid_t(y1, y2, y3);

    //center of triangle 2 3 4

    float centroid_t2_x = centroid_t(x2, x3, x4);
    float centroid_t2_y = centroid_t(y2, y3, y4);


    //center of triangle 1 3 4

    float centroid_t3_x = centroid_t(x1, x3, x4);
    float centroid_t3_y = centroid_t(y1, y3, y4);


    // center of triangle 1 2 4

    float centroid_t4_x = centroid_t(x1, x2, x4);
    float centroid_t4_y = centroid_t(y1, y2, y4);


    float quadcenter_x  = centroid_t1_x + centroid_t2_x + centroid_t3_x + centroid_t4_x;


}





#endif //DE_CENTROID_H
