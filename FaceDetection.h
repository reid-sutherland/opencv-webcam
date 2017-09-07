//
// Created by reid on 9/6/17.
//

#ifndef OPENCV_WEBCAM_FACEDETECTION_H
#define OPENCV_WEBCAM_FACEDETECTION_H

//OpenCV
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/tracking/tracking.hpp"

//Local
#include "StereoCalibration.h"
#include "Disparity.h"

//System
#include <string>
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

class FaceDetection {

private:

    StereoCalibration& stereoCalibration;
    Disparity& disp;

    CascadeClassifier face_cascade;
    CascadeClassifier eyes_cascade;
    string face_cascade_name = "/home/reid/opencv-webcam/data/haarcascade_frontalface_alt.xml";
    string eyes_cascade_name = "/home/reid/opencv-webcam/data/haarcascade_eye_tree_eyeglasses.xml";

    vector<Rect> rectFaces;
    vector<Point> centers;
    Mat m_frameGray;
    Mat m_frameLeft;
    Mat m_frameRight;
    Rect faceROI, smallFaceROI;

public:

    explicit FaceDetection(Disparity& disparity);

    int detectFace(Mat frameLeft, Mat frameRight, Mat& outputFrame);
    void trackFace();
    int computeDistanceToFace(Mat& left_face);
    bool loadCascades();

    // Getters
    vector<Rect> getFaces() { return rectFaces; };
    vector<Point> getCenters() { return centers; };

};


#endif //OPENCV_WEBCAM_FACEDETECTION_H
