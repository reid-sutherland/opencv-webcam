/*
 * StereoRectification.cpp
 *
 *  Created on: 20 Jan. 2016
 *  Author: erman
 */

#include "StereoRectification.h"

StereoRectification::StereoRectification(std::string retroProjectionMatFilename)
    : retroProjectionMatFilename(retroProjectionMatFilename) {
    //loadQMatrice();
}

StereoRectification::~StereoRectification(){
    // TODO Auto-generated destructor stub
}

/**
 * Cette fonction permet de sauvegarder dans un fichier les parametres resultants de la rectification dans un fichier.
*/
int StereoRectification::saveQMatrice()
{
    cv::FileStorage fs(this->retroProjectionMatFilename, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        std::cout << this->retroProjectionMatFilename << " could not be opened\n";
        return -1;
    }
    else
    {
        fs << "Q" << Q;
        fs.release();
        //ccLog::Print("rectification matrices saved to %s\n", this->retroProjectionMatFilename.c_str());
    }
    return 0;
}


/**
 * Cette fonction permet de lire dans un fichier les parametres de la rectification dans un fichier.
*/
int StereoRectification::loadQMatrice()
{
    std::cout<<"=============in loadQMatrice function file==================\n";
    cv::FileStorage fs(this->retroProjectionMatFilename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "Camera rectification file : " << this->retroProjectionMatFilename << " is not found\n";
        return -1;
    }
    else
    {
        fs["Q"] >> this->Q;
        fs.release();
        //ccLog::Print("rectification matrices successfully loaded from %s.\n", this->retroProjectionMatFilename.c_str());
        double Q03, Q13, Q23, Q32, Q33;
        Q03 = Q.at<double>(0,3);
        Q13 = Q.at<double>(1,3);
        Q23 = Q.at<double>(2,3);
        Q32 = Q.at<double>(3,2);
        Q33 = Q.at<double>(3,3);
        std::cout << "loaded function: Q(0,3) = "<< Q03 <<"; Q(1,3) = "<< Q13 <<"; Q(2,3) = "<< Q23 <<"; Q(3,2) = "<< Q32 <<"; Q(3,3) = "<< Q33 <<";" << std::endl;

    }

    return 0;
}

void StereoRectification::rectify(StereoCalibration &sc, bool saveQMatrice)
{
    sc.loadCalib();

//    //Rectification
//    cv::Mat RotMat_leftCam, RotMat_rightCam, ProjMat_leftCam, ProjMat_rightCam;

//    //! Stereo rectification  using Bouguet Algorithm
//    //! The output of this operation is: RotMat_leftCam(rotation matrix of left cam), RotMat_rightCam(rotation matrix of right cam)
//    //! ProjMat_leftCam: (Projection matrix of left cam), ProjMat_rightCam(Projection Matrix of right cam), Q is and optional parameter.
//    cv::stereoRectify(sc.CM1, sc.D1, sc.CM2,
//                      sc.D2, sc.framesize, sc.R, sc.T,
//                      RotMat_leftCam, RotMat_rightCam, ProjMat_leftCam,
//                      ProjMat_rightCam, this->Q, 0, -1, sc.framesize,
//                      &this->validRoi[0], &this->validRoi[1]);

//    ccLog::Print("After stereoRectify .\n");
//    //! Generate maps for undistorted and rectified images
//    //! These maps(map1x, map1y) indicate from where we should interpolate source pixels for each pixel of the
//    //! destination image; the maps can then be plugged directly into cvRemap(),
//    cv::initUndistortRectifyMap(sc.CM1, sc.D1, RotMat_leftCam,
//                                ProjMat_leftCam, sc.framesize, CV_32FC1,
//                                this->map1x, this->map1y);

//    cv::initUndistortRectifyMap(sc.CM2, sc.D2, RotMat_rightCam,
//                                ProjMat_rightCam, sc.framesize, CV_32FC1,
//                                this->map2x, this->map2y);

//    std::cout << "sc.framesize : " << sc.framesize << " - map1y : " << this->map1y.cols << " - sc.CM1 : " << sc.CM1 << " - map2y : " << this->map2y.cols << std::endl;
//    std::cout << "Q(0,3) = "<< Q.at<double>(0,3)
//                <<"; Q(1,3) = "<< Q.at<double>(1,3)
//                <<"; Q(2,3) = "<< Q.at<double>(2,3)
//                <<"; Q(3,2) = "<< Q.at<double>(3,2)
//                <<"; Q(3,3) = "<< Q.at<double>(3,3) <<";" << std::endl;

//    if(saveQMatrice)
//    {
//        this->saveQMatrice();
//    }
    //ccLog::Print("Rectification complete.");
}


void StereoRectification::rectifyStereoImg(StereoCalibration& sc, cv::Mat imgLeft, cv::Mat imgRight, cv::Mat& imgLeftRect, cv::Mat& imgRightRect, cv::Mat& Q)
{
//    cv::imshow("Before retify(imLeft)", imgLeft);
//    cv::imshow("Before retify(imRight)", imgRight);
    rectify(sc);

    //ccLog::Print("Applying of rectification on images.\n");
    //! Applaying rectification of images
    cv::remap(imgLeft, imgLeftRect, this->map1x, this->map1y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::remap(imgRight, imgRightRect, this->map2x, this->map2y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
//    cv::imshow("After retify(imLeft)", imgLeft);
//    cv::imshow("After retify(imRight)", imgRight);
//    cv::waitKey(30);
}

void StereoRectification::rectifyStereoImg(StereoCalibration& sc, cv::Mat imgLeft, cv::Mat imgRight, cv::Mat& imgLeftRect, cv::Mat& imgRightRect)
{
    rectifyStereoImg(sc, imgLeft, imgRight, imgLeftRect, imgRightRect, this->Q);

}












