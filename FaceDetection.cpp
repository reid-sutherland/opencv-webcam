//
// Created by reid on 9/6/17.
//

//OpenCV

//Local
#include "FaceDetection.h"

//System

FaceDetection::FaceDetection(Disparity &disparity) :
        disp(disparity), stereoCalibration(StereoCalibration::Instance())
{

}

int FaceDetection::detectFace(Mat frameLeft, Mat frameRight, Mat& outputFrame) {
    // Detect empty frames
    if (frameLeft.empty()) {
        cout << "[FaceDetection] Error reading frame." << endl;
        return -1;
    }

    centers = vector<Point>();
    m_frameLeft = frameLeft;
    m_frameRight = frameRight;

    outputFrame = frameLeft.clone();

    cvtColor(outputFrame, m_frameGray, CV_BGR2GRAY);
    equalizeHist(m_frameGray, m_frameGray);

    // Detect Faces
    face_cascade.detectMultiScale(m_frameGray, rectFaces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30));

    for (size_t i = 0; i < rectFaces.size(); i++) {
        Point center(rectFaces[i].x + rectFaces[i].width*0.5, rectFaces[i].y + rectFaces[i].height*0.5);
        centers.push_back(center);

        rectangle(outputFrame, rectFaces[i], Scalar(255, 0, 0), 4);
    }

    return 0;
}

void FaceDetection::trackFace() {
    if (rectFaces.size() > 1) {
        cout << "[trackFace] Error: Too many faces to track, 1 required." << endl;
        return;
    }


}

int FaceDetection::computeDistanceToFace(Mat& left_face) {
    if (rectFaces.empty() || centers.empty()) {
        cout << "[computeDistanceToFace] Vector error." << endl;
        return -1;
    }

    // Compute disparity map
    disp.computeDispMap(m_frameLeft, m_frameRight);
    Mat disparity = disp.filtered_disp;
    Mat disp_face = disp.filtered_disp.clone();

    // Draw matching rectangles onto disp_face
    rectangle(disp_face, rectFaces[0], Scalar(255, 0, 0), 4);
    smallFaceROI = Rect(rectFaces[0].x + (rectFaces[0].width*0.25),
                          rectFaces[0].y + (rectFaces[0].height*0.25),
                          rectFaces[0].width*0.5,
                          rectFaces[0].height*0.5);
    rectangle(disp_face, smallFaceROI, Scalar(0, 0, 255), 4);

    while (waitKey(50) != 27) {
        imshow("left_face", left_face);
        imshow("disp_face", disp_face);
    }
    destroyWindow("left_face");
    destroyWindow("disp_face");

    Mat rgbFaceFrame, dispFaceFrame;
    // if shit hits the fan, change m_frameLeft to a clone
    rgbFaceFrame = Mat(m_frameLeft, smallFaceROI);
    dispFaceFrame = Mat(disparity, smallFaceROI);

    // Find the average disparity across the ROI
    int sum = 0;
    int d_int = 0;
    int numValues = 0;
    for (int i = 0; i < rgbFaceFrame.rows; i++) {
        auto* rgb_row_ptr = rgbFaceFrame.ptr<uchar>(i);
        auto* disp_row_ptr = dispFaceFrame.ptr<uchar>(i);

        for (int j = 0; j < rgbFaceFrame.cols; j++) {
            // add each disp value to the sum
            uchar d = disp_row_ptr[j];
            d_int = (int) d;
            sum += d_int;

            // test - check first disparity value
            if (numValues == 0) {
                cout << "first disp value = " << d_int << endl;
            }

            // increment numValues
            numValues++;
        }
    }
    // Calculate the average disparity
    double avgDisp = sum / numValues;
    cout << "avgDisp = " << avgDisp << endl;

    // Use avgDisp with the Q matrix to calculate distance
    double distance, baseline, focal;
    Mat Q = stereoCalibration.getQMatrix();
    baseline = -1.0/Q.at<double>(3, 2);
    if (baseline < 0) { baseline *= -1.0;}  // if baseline is negative, make it positive
    focal = Q.at<double>(2, 3);

    distance = (baseline * focal) / avgDisp;
    cout << "Distance to face is: " << distance << " meters?" << endl;
}

bool FaceDetection::loadCascades() {
    // Load the cascade
    if (!face_cascade.load(face_cascade_name)) {
        cout << "[FaceDetection] Error loading face cascade." << endl;
        return false;
    } else if (!eyes_cascade.load(eyes_cascade_name)) {
        cout << "[FaceDetection] Error loading eyes cascade." << endl;
        return false;
    } else {
        cout << "[FaceDetection] Successfully loaded cascade." << endl;
        return true;
    }
}

/*
D:= Distance of point in real world,
b:= base offset, (the distance *between* your cameras)
f:= focal length of camera,
d:= disparity:

D = b*f/d
*/