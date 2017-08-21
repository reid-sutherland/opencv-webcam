#include "Reconstruction3D.h"

//#define DEBUG
//#define TestVIZ
#ifdef TestVIZ
#include <opencv2/viz/vizcore.hpp>
#endif


Reconstruction3D::Reconstruction3D(Disparity& disparity)
    :disp(disparity), maxDepth(1000.0), alpha(5.0), handleMissingValues(true), bufsize(1),
     point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>)
{
}

Reconstruction3D::~Reconstruction3D() {
	// TODO Auto-generated destructor stub
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

    //! Rectification of images
    cout << "Starting rectification...";
    sc.rectifyStereoImg(img_rgb_left, img_rgb_right, img_rgb_left, img_rgb_right);
#ifdef DEBUG
    cv::imshow("Rectify Image L", img_rgb_left);
    cv::imshow("Rectify Image R", img_rgb_right);
#endif
    cout << "\tEnd of rectification process." << endl;

    //! Computing of disparity Map
    //Disparity disp("SGBM");
    cout << "Starting disparity...";

    if (disp.computeNormDisp(img_rgb_left, img_rgb_right) < 0 )
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
    cout << "\t\tEnd of disparity." << endl;

    //! Declaration of 3D reconstruction parameter
    cv::Mat Q = sc.getQMatrix();
    cout << "Getting Q matrix..." << endl;

    //! Reprojection 3D
    cout << "Starting 3D reprojection...";
#ifdef TestVIZ
    cv::Mat XYZ(img_disparity.size(), CV_32FC3);
    cv::reprojectImageTo3D(img_disparity, XYZ, Q);

       //----------------------------------------------------------------------
       // Visualize
       //----------------------------------------------------------------------

       /// Create a window
       /// cv::viz::Viz3d window = cv::viz::Viz3d("Viz demonstration");
       cv::viz::Viz3d myWindow("Coordinate Frame");

       while (!myWindow.wasStopped())
       {
           /// Create a cloud widget
           cv::viz::WCloud cw(XYZ, cv::viz::Color::red());

           /// Display it in a window
           myWindow.showWidget("CloudWidget1", cw);

           myWindow.spinOnce(1, true);
       }
#endif
    CustomProject3d(Q, img_rgb_left, img_disparity, point_cloud_ptr);
    cout << "\tEnd of 3D reprojection." << endl;

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
        auto* rgb_ptr = img_rgb.ptr<uchar>(i);
        auto* disp_ptr = img_disparity.ptr<uchar>(i);

        for (int j = 0; j < img_rgb.cols; j++)
        {
            //Get 3D coordinates
            uchar d = disp_ptr[j];

            //if (d == 0) continue; //Discard bad pixels
            if(d < FLT_EPSILON || d > max_z) continue; //Discard bad pixels
            double pw = -1.0 * static_cast<double>(d)* (Q.at<double>(3, 2) + Q.at<double>(3, 3));
            px = static_cast<double>(j)+Q.at<double>(0, 3);
            py = static_cast<double>(i)+Q.at<double>(1, 3);
            pz = Q.at<double>(2, 3);

            px = px / pw;
            py = py / pw;
            pz = pz / pw;

            //Get RGB info
            pb = rgb_ptr[3 * j + 0];
            pg = rgb_ptr[3 * j + 1];
            pr = rgb_ptr[3 * j + 2];

            //Insert info into point cloud structure
            pcl::PointXYZRGB point;
            point.x = px;
            point.y = py;
            point.z = pz;
            uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
                            static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->points.push_back(point);
        }
    }
    point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;
    return 0;
}

void Reconstruction3D::printParameter()
{ 
    printf(" --- Reconstruction3D Parameters --- \n");
}
