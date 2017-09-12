//
// Created by reid on 9/6/17.
//

#ifndef OPENCV_WEBCAM_FACEDETECTION_H
#define OPENCV_WEBCAM_FACEDETECTION_H

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/core/utility.hpp>
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
    //! General
    StereoCalibration* stereoCalibration;
    Disparity& disp;
    Mat m_frameLeft, m_frameRight, m_outputFrame;
    Mat Q;
    double m_distance;

    //! Object Tracking
    Ptr<Tracker> m_tracker;
    Rect2d m_trackerROI;

    //! Face Detection
    CascadeClassifier face_cascade;
    CascadeClassifier eyes_cascade;
    string face_cascade_name = "/home/reid/opencv-webcam/data/haarcascade_frontalface_alt.xml";
    string eyes_cascade_name = "/home/reid/opencv-webcam/data/haarcascade_eye_tree_eyeglasses.xml";
    vector<Rect> rectFaces;
    vector<Point> centers;
    Mat m_frameGray;
    Rect m_faceROI, m_smallFaceROI;
    Size m_minSize;

public:
    //! Constructor
    explicit FaceDetection(Disparity& disparity);

    //! Destructor
    virtual ~FaceDetection();

    //! Object Tracking
    bool initializeTrackerROI(Mat frameLeft);
    int trackFace(Mat frameLeft, Mat frameRight);
    int computeDistanceToObject(bool computeDisp=false);

    //! Face Detection
    int detectFace(Mat frameLeft, Mat frameRight);
    int computeDistanceToFace(bool computeDisp=false);
    Rect findLargestFaceROI(vector<Rect> rectFaces);
    bool loadCascades();

    //! Misc.
    void drawDistance();

    //! Getters
    vector<Rect> getFaces() { return rectFaces; };
    vector<Point> getCenters() { return centers; };
    Mat outputFrame() { return m_outputFrame; };

};


#endif //OPENCV_WEBCAM_FACEDETECTION_H
