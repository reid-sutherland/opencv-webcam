#include "Reconstruction3D.h"

//#define DEBUG
//#define TestVIZ
#ifdef TestVIZ
#include <opencv2/viz/vizcore.hpp>
#include <thread>

#endif


Reconstruction3D::Reconstruction3D(Disparity& disparity)
    : disp(disparity), maxDepth(1000.0), alpha(5.0), handleMissingValues(true), bufsize(1),
     point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    print = true;
}

Reconstruction3D::~Reconstruction3D() {

}


int Reconstruction3D::buildPointCloud(cv::Mat& img_rgb_left, cv::Mat& img_rgb_right, StereoCalibration& sc)
{
    if(img_rgb_left.size() != sc.getFrameSize())
    {
        printf("[buildPointCloud] left image size(%d, %d) ~ nativeSize(%d %d) \n",
                         img_rgb_left.size().width, img_rgb_left.size().height,
                         sc.getFrameSize().width, sc.getFrameSize().height);
        cv::resize(img_rgb_left, img_rgb_left, sc.getFrameSize());
    }

    if(img_rgb_right.size() != sc.getFrameSize())
    {
        printf("[buildPointCloud] right image size(%d, %d) ~ nativeSize(%d %d) \n",
                         img_rgb_right.size().width, img_rgb_right.size().height,
                         sc.getFrameSize().width, sc.getFrameSize().height);
        cv::resize(img_rgb_right, img_rgb_right, sc.getFrameSize());
    }

    //! Computing of disparity Map
    if (disp.computeDispMap(img_rgb_left, img_rgb_right) < 0 )
    {
        cerr << "Error: [buildPointCloud] Internal error on disparity process." << endl;
        return -1;
    }
    cv::Mat img_disparity = disp.filtered_disp;

#ifdef DEBUG
    cv::imshow("Disparity_Image", img_disparity);
    cv::imshow("disp.raw_disp", disp.raw_disp);
    cv::waitKey(100);
#endif

    //! Declaration of 3D reconstruction parameter
    cv::Mat Q = sc.getQMatrix();

    //! Reprojection 3D
#ifdef TestVIZ
    cv::Mat XYZ(img_disparity.size(), CV_32FC3);
    cv::reprojectImageTo3D(img_disparity, XYZ, Q);

    while (cv::waitKey(10) != 27) {
        //cv::imshow("test disp window", img_disparity);
        cv::imshow("test xyz window", XYZ);
    }

    //Visualize
    // Create a window
    cv::viz::Viz3d myWindow("Coordinate Frame");

    cv::viz::WCloud cw(XYZ, cv::viz::Color::red());
    myWindow.showWidget("CloudWidget1", cw);
    myWindow.spin();


    return 0;
    myWindow.spinOnce(1, true);
    while (!myWindow.wasStopped()) {
        // Create a cloud widget
        cv::viz::WCloud cw(XYZ, cv::viz::Color::red());

        // Display it in a window
        myWindow.showWidget("CloudWidget1", cw);

        myWindow.spinOnce(1, true);
    }

    cout << "Last event loop is over" << endl;

#endif

    //CustomProject3d(Q, img_rgb_left, img_disparity, point_cloud_ptr);

    return 0;
}

int Reconstruction3D::CustomProject3d(cv::Mat Q, cv::Mat img_rgb, cv::Mat img_disparity,
                                      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr)
{
   //Create point cloud and fill it
    double px, py, pz;
    uchar pr, pg, pb;
    const double max_z = 1.0e4;

    for (int i = 0; i < img_rgb.rows; i++)
    {
        auto* rgb_row_ptr = img_rgb.ptr<uchar>(i);
        auto* disp_row_ptr = img_disparity.ptr<uchar>(i);

        for (int j = 0; j < img_rgb.cols; j++)
        {
            //Get 3D coordinates
            uchar d = disp_row_ptr[j];

            if(d < FLT_EPSILON || d > max_z) { continue; } //Discard bad pixels

            // pw = <double> d / Tx
            double pw = -1.0 * static_cast<double>(d) * (Q.at<double>(3, 2) + Q.at<double>(3, 3));
            px = static_cast<double>(j)+Q.at<double>(0, 3);     //px = col# + (-cx)
            py = static_cast<double>(i)+Q.at<double>(1, 3);     //py = row# + (-cy)
            pz = Q.at<double>(2, 3);                            //pz = focal pt

            px = px / pw;
            py = py / pw;
            pz = pz / pw;

            //Get RGB info
            pb = rgb_row_ptr[3 * j + 0];
            pg = rgb_row_ptr[3 * j + 1];
            pr = rgb_row_ptr[3 * j + 2];

            //Insert info into point cloud structure
            pcl::PointXYZRGB point;
            point.x = (float) px;
            point.y = (float) py;
            point.z = (float) pz;
            uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
                            static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
            point.rgb = *reinterpret_cast<float*>(&rgb);
                /*using static_cast results in the error: invalid static_cast
                  from type ‘uint32_t* {aka unsigned int*}’ to type ‘float*’ */
            point_cloud_ptr->points.push_back(point);
        }
    }
    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    return 0;
}

void Reconstruction3D::printQMatrix(cv::Mat Q) {
    cout << "Cx = " << -1*Q.at<double>(0, 3) << endl;
    cout << "Cy = " << -1*Q.at<double>(1, 3) << endl;
    cout << "f = " << Q.at<double>(2, 3) << endl;
    double Tx = -1/Q.at<double>(3, 2);
    cout << "Tx = " << Tx << endl;
    cout << "(Cx - C'x) = " << Tx*Q.at<double>(3, 3) << endl << endl;
}

void Reconstruction3D::printParameter()
{
    std::cout << " --- Reconstruction3D Parameters --- " << std::endl;
}


/*

Q Matrix form:

    | 1  0    0         -cx      |
Q = | 0  1    0         -cy      |
    | 0  0    0          f       |
    | 0  0  -1/Tx  (cx - c'x)/Tx |

    cx / cy     coordinates of the principal point in the dominant camera
    c'x         x-coordinate of the principal point in the non-dominant camera
                    (will be equal to cx if CALIB_ZERO_DISPARITY is specified, which is normally the case here)
    f           focal length
    Tx          baseline length - translation from one optical center to the other

    Orientation: cx is at (0,3), cy is at (1,3), etc.
*/




