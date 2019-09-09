//
// Created by asif on 09.09.19.
//


#include "../include/GLViewer.hpp"
#include "sl/Camera.hpp"
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


using namespace sl;
using namespace std;


int main(int argc, char* argv[] ) {



    Camera zed;
    // Set configuration parameters for the ZED
    InitParameters initParameters;
    initParameters.depth_mode = DEPTH_MODE_ULTRA;
    initParameters.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // OpenGL's coordinate system is right_handed
    // open SVO if one given as parameter
    if(argc > 1 && string(argv[1]).find(".svo"))
        initParameters.svo_input_filename = argv[1];

    // Open the camera
    ERROR_CODE zed_error = zed.open(initParameters);

    if(zed_error != SUCCESS) {// Quit if an error occurred
        cout << zed_error << endl;
        zed.close();
        return 1;
    }

    Resolution resolution = zed.getResolution();
    CameraParameters camera_parameters = zed.getCameraInformation().calibration_parameters.left_cam;

    // Point cloud viewer
    GLViewer viewer;
    // Initialize point cloud viewer
    viewer.init(argc, argv, camera_parameters);

    // Allocation of 4 channels of float on GPU
    Mat point_cloud(resolution, MAT_TYPE_32F_C4, MEM_GPU);
    Mat ImageLeft;


    // Main Loop
    while(viewer.isAvailable()) {
        if(zed.grab() == SUCCESS) {
            zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA, MEM_GPU);
            // zed.retrieveMeasure(ImageLeft, VIEW_LEFT);
            viewer.updatePointCloud(point_cloud);
        } else sleep_ms(1);
    }
    // free allocated memory before closing the ZED
    point_cloud.free();

    // close the ZED
    zed.close();

    return 0;


}
