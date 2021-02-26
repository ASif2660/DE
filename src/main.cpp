
#Author : Asif

#include <sl/Camera.hpp>

// Sample includes
//#include "aruco.hpp"

#include "opencv2/aruco.hpp"
#include <string>
#include <vector>


// OCV includes
#include <opencv2/opencv.hpp>
#include <birdsEye.h>

using namespace sl;
using namespace std;

const int MAX_CHAR = 128;  // IMU


int main(int argc, char **argv) {



    // Create a ZED camera object

    Camera zed;



    // Set configuration parameters

    InitParameters init_params;

    init_params.camera_resolution = RESOLUTION_HD720;

    init_params.coordinate_units = UNIT_MILLIMETER;

    init_params.camera_disable_imu = true; // for this sample, IMU (of ZED-M) is disable, we use the gravity given by the marker.


    init_params.coordinate_system = COORDINATE_SYSTEM_LEFT_HANDED_Y_UP;

    // Open the camera

    ERROR_CODE err = zed.open(init_params);

    if (err != SUCCESS) {

        cout << "Error, unable to open ZED camera: " << err << "\n";

        zed.close();

        return 1; // Quit if an error occurred

    }


    // for position of camera


    // IMU CODE






    Resolution image_size = zed.getResolution();

    Mat image_zed(image_size, MAT_TYPE_8U_C4);

    cv::Mat image_ocv = cv::Mat(image_zed.getHeight(), image_zed.getWidth(), CV_8UC4, image_zed.getPtr<sl::uchar1>(MEM_CPU));

    cv::Mat image_ocv_rgb;



    auto calibInfo = zed.getCameraInformation().calibration_parameters.left_cam;

    cv::Matx33d camera_matrix = cv::Matx33d::eye();

    camera_matrix(0, 0) = calibInfo.fx;

    camera_matrix(1, 1) = calibInfo.fy;

    camera_matrix(0, 2) = calibInfo.cx;

    camera_matrix(1, 2) = calibInfo.cy;


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

    float fx_camera = calibInfo.fx;

    float fy_camera = calibInfo.fy;

    cv::Mat warped_image;

    cv::Mat lane_image;

    // Create text for GUI
    char text_rotation[MAX_CHAR];

    char text_translation[MAX_CHAR];

    cv::Matx<float, 4, 1> dist_coeffs = cv::Vec4f::zeros();

    float actual_marker_size_meters = 0.16f; // real marker size in meters

    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);

    cout << " Is the ARUCO Marker 6x6 ?, check " << actual_marker_size_meters * 1000 << " mm" << endl;

    Transform pose;

    Pose zed_pose;

    vector<cv::Vec3d> rvecs, tvecs;

    vector<int> ids;

    vector<vector<cv::Point2f> > corners;

    string position_txt;

    sl::Mat point_cloud;

    sl::float4 point_cloud_value;

    bool can_reset = false;

    zed.enableTracking();

    // Loop until 'q' is pressed

    char key = '.';

    while (key != 'q') {

        if (zed.grab() == SUCCESS) {

            // Retrieve the left image

            zed.retrieveImage(image_zed, VIEW_LEFT, MEM_CPU, image_size.width, image_size.height);

            // convert to RGB
            cv::cvtColor(image_ocv, image_ocv_rgb, cv::COLOR_RGBA2RGB);
            // detect marker
            cv::aruco::detectMarkers(image_ocv_rgb, dictionary, corners, ids);
            // get actual ZED position
            zed.getPosition(zed_pose);

            zed.retrieveMeasure(point_cloud, MEASURE_XYZBGRA);
            // display ZED position
            cv::rectangle(image_ocv_rgb, cv::Point(0, 0), cv::Point(490, 75), cv::Scalar(0, 0, 0), -1);

            cv::putText(image_ocv_rgb, "Loaded dictionary : 6x6.     Press 'SPACE' to reset the camera "
                                       "position", cv::Point(10, 15), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(220, 220, 220));

            sl::float3 zed_rdy = zed_pose.pose_data.getEulerAngles(false);

            position_txt =
                    "yaw zed: " + to_string(zed_rdy[2]) +
                    " ZED  x: " + to_string(zed_pose.pose_data.tx) + "; y: " + to_string(zed_pose.pose_data.ty) + "; z: " + to_string(zed_pose.pose_data.tz);

            cv::putText(image_ocv_rgb, position_txt, cv::Point(10, 35), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(236, 188, 26));


            // if at least one marker detected

            if (ids.size() > 0) {

                cv::aruco::estimatePoseSingleMarkers(corners, actual_marker_size_meters, camera_matrix, dist_coeffs, rvecs, tvecs);

                pose.setTranslation(sl::float3(tvecs[0](0), tvecs[0](1), tvecs[0](2)));

                pose.setRotationVector(sl::float3(rvecs[0](0), rvecs[0](1), rvecs[0](2)));

                auto rot_matrix = pose.getRotationMatrix();

                //pose.inverse();

                can_reset = true;

                cv::aruco::drawDetectedMarkers(image_ocv_rgb, corners, ids);

                cv::aruco::drawAxis(image_ocv_rgb, camera_matrix, dist_coeffs, rvecs[0], tvecs[0], actual_marker_size_meters * 0.5f);

                //position_txt = "Aruco x: " + to_string(pose.tx) + "; y: " + to_string(pose.ty) + "; z: " + to_string(pose.tz);

                int x = (corners[0][0].x + corners[0][1].x + corners[0][2].x + corners[0][3].x) / 4;
                int y = (corners[0][0].y + corners[0][1].y + corners[0][2].y + corners[0][3].y) / 4;


                sl::float3 yrp = pose.getEulerAngles(false);

             //     surroundEye.scalingFactor(corners, zed.getResolution());

                point_cloud.getValue(x, y, &point_cloud_value);

                float distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);

                position_txt =
                        "distance z-axis: " + to_string(pose.tx*100) +
                        " X: " + to_string(x) + " Y: " + to_string(y) +  " Distance is: " + to_string(distance);


                cv::putText(image_ocv_rgb, position_txt, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(124, 252, 124));


//                std::cout << image_ocv_rgb.size <<"the size of the image " << std::endl;

            } else

                can_reset = false;


            cv::Mat final_image = image_ocv_rgb;

            birdsEye surroundEye(image_ocv_rgb, fx_camera, fy_camera);

            img_denoise = surroundEye.deNoise(image_ocv_rgb);

            img_edges = surroundEye.edgeDetector(img_denoise);

            img_mask = surroundEye.mask(img_edges);

            lines = surroundEye.houghLines(img_mask);

           // cv::Mat top_view = surroundEye.birdsEyeFunction(image_ocv_rgb);




            if (!lines.empty()) {
                // Separate lines into left and right lines
                left_right_lines = surroundEye.lineSeparation(lines, img_edges);

                // Apply regression to obtain only one line for each side of the lane
                lane = surroundEye.regression(left_right_lines, image_ocv_rgb);

                // Predict the turn by determining the vanishing point of the the lines
                turn = surroundEye.predictTurn();

                // Plot lane detection
//                lane_image = surroundEye.plotLane(image_ocv_rgb, lane, turn);

  //              warped_image = surroundEye.birdsEyeFunction(lane_image);


                iterator += 1;


            }

       /*     std::cout << warped_image.size << std::endl;
            std::cout << final_image.size << std::endl;
*/

           //cv::imshow("Image", warped_image);
           cv::imshow("ActualImage", final_image);

            key = cv::waitKey(10);


            // Handle key event

            // if KEY_R is pressed and aruco marker is visible, then reset ZED position

            if (key == ' ')

                zed.resetTracking(pose);

        }

    }

    zed.close();

    return 0;

}
