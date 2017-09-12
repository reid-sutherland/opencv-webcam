//
// Created by reid on 9/6/17.
//

//#define DEBUG

//Local
#include "FaceDetection.h"

FaceDetection::FaceDetection(Disparity &disparity) :
        disp(disparity), stereoCalibration(StereoCalibration::instance())
{

}

FaceDetection::~FaceDetection() {

}

bool FaceDetection::initializeTrackerROI(Mat frameLeft) {
    // Create a Tracker object
    m_tracker = TrackerKCF::create();   // Kernelized Correlation Filters

    // Select ROI
    //bool fromCenter = false;        // we want to drag from top left to bottom right, not from center out
    m_trackerROI = selectROI("tracker", frameLeft /*, fromCenter*/);

    // Crop image
    Mat frameCrop = frameLeft(m_trackerROI);

    // Display Cropped Image
    while (waitKey(50) != 27) {
        if (!frameCrop.empty()) {
            imshow("Cropped", frameCrop);
        }
    }
    destroyWindow("Cropped");

    if (m_trackerROI.width == 0 || m_trackerROI.height == 0) {
        cout << "[initializeTrackerROI] Error: Invalid ROI." << endl;
        return false;
    } else {
        // Initialize Tracker
        m_tracker->init(frameLeft, m_trackerROI);
        return true;
    }
}

int FaceDetection::trackFace(Mat frameLeft, Mat frameRight) {
    //! Break if empty images
    if (frameLeft.empty() || frameRight.empty()) {
        cout << "[trackFace] Error: Empty frame(s)." << endl;
        return -1;
    }

    //! Get frames
    m_frameLeft = frameLeft;
    m_frameRight = frameRight;
    m_outputFrame = frameLeft.clone();

    //! Update the tracking result
    m_tracker->update(frameLeft, m_trackerROI);

    //! Draw the tracked object
    rectangle(m_outputFrame, m_trackerROI, Scalar(255, 0, 0), 2, 1);
}

int FaceDetection::computeDistanceToObject(bool computeDisp) {
    if (computeDisp) {
        // Compute disparity map
        disp.computeDispMap(m_frameLeft, m_frameRight);
    }

    Mat dispFaceFrame;
    dispFaceFrame = Mat(disp.filtered_disp, m_trackerROI);

    // Find the average disparity across the ROI
    int sum = 0, d_int = 0, numValues = 0;
    for (int i = 0; i < dispFaceFrame.rows; i++) {
        //auto* rgb_row_ptr = rgbFaceFrame.ptr<uchar>(i);   //not needed
        auto* disp_row_ptr = dispFaceFrame.ptr<uchar>(i);

        for (int j = 0; j < dispFaceFrame.cols; j++) {
            // add each disp value to the sum
            uchar d = disp_row_ptr[j];
            d_int = (int) d;
            sum += d_int;

            // increment numValues
            numValues++;
        }
    }
    // Calculate the average disparity
    double avgDisp = sum / numValues;

    // Use avgDisp with the Q matrix to calculate distance
    double baseline, focal;

    baseline = -1.0/Q.at<double>(3, 2);
    if (baseline < 0) { baseline *= -1.0;}  // if baseline is negative, make it positive

    focal = Q.at<double>(2, 3);

    m_distance = (baseline * focal) / avgDisp;

#ifdef DEBUG
    stereoCalibration->printQMatrix();
    cout << "avgDisp = " << avgDisp << endl;
    cout << "baseline = " << baseline << endl;
    cout << "focal = " << focal << endl;
    cout << "\n***Distance to object is: " << m_distance << " meters***" << endl << endl;
#endif

    return 0;
}

int FaceDetection::detectFace(Mat frameLeft, Mat frameRight) {
    // Detect empty frames
    if (frameLeft.empty() || frameRight.empty()) {
        cout << "[FaceDetection] Error reading frame." << endl;
        return -1;
    }

    centers = vector<Point>();
    m_frameLeft = frameLeft;
    m_frameRight = frameRight;
    m_outputFrame = frameLeft.clone();
    m_minSize = Size(30, 30);

    cvtColor(m_outputFrame, m_frameGray, CV_BGR2GRAY);
    equalizeHist(m_frameGray, m_frameGray);

    // Detect Faces
    face_cascade.detectMultiScale(m_frameGray, rectFaces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, m_minSize);

    m_faceROI = findLargestFaceROI(rectFaces);
    if (m_faceROI.area() >= m_minSize.area()) {
        rectangle(m_outputFrame, m_faceROI, Scalar(255, 0, 0), 4);
    }

    return 0;
}

int FaceDetection::computeDistanceToFace(bool computeDisp) {
    if (m_faceROI.area() < m_minSize.area()) {
        cout << "[computeDistanceToFace] m_faceROI < m_minSize.area()" << endl;
        return -1;
    }

    if (computeDisp) {
        // Compute disparity map
        disp.computeDispMap(m_frameLeft, m_frameRight);
    }
    Mat disparity = disp.filtered_disp;

    // Calculate smallFaceROI and draw it on the frame
    m_smallFaceROI = Rect(m_faceROI.x + m_faceROI.width/4,
                          m_faceROI.y + m_faceROI.height/4,
                          m_faceROI.width/2,
                          m_faceROI.height/2);
    rectangle(m_outputFrame, m_smallFaceROI, Scalar(0, 0, 255), 4);


    Mat dispFaceFrame;
    dispFaceFrame = Mat(disparity, m_smallFaceROI);

    // Find the average disparity across the ROI
    int sum = 0, d_int = 0, numValues = 0;
    for (int i = 0; i < dispFaceFrame.rows; i++) {
        //auto* rgb_row_ptr = rgbFaceFrame.ptr<uchar>(i);   //not needed
        auto* disp_row_ptr = dispFaceFrame.ptr<uchar>(i);

        for (int j = 0; j < dispFaceFrame.cols; j++) {
            // add each disp value to the sum
            uchar d = disp_row_ptr[j];
            d_int = (int) d;
            sum += d_int;

            // increment numValues
            numValues++;
        }
    }
    // Calculate the average disparity
    double avgDisp = sum / numValues;

    // Use avgDisp with the Q matrix to calculate distance
    double baseline, focal;

    baseline = -1.0/Q.at<double>(3, 2);
    if (baseline < 0) { baseline *= -1.0;}  // if baseline is negative, make it positive

    focal = Q.at<double>(2, 3);

    m_distance = (baseline * focal) / avgDisp;

#ifdef DEBUG
    stereoCalibration->printQMatrix();
    cout << "avgDisp = " << avgDisp << endl;
    cout << "baseline = " << baseline << endl;
    cout << "focal = " << focal << endl;
    cout << "\n***Distance to face is: " << m_distance << " meters***" << endl << endl;
#endif

    return 0;
}

void FaceDetection::drawDistance() {
    // Create the distance string
    string distanceStr = "Distance: ";
    distanceStr.append(to_string(m_distance));
    distanceStr.append(" (m)");

    // Initialize textDraw values
    int fontFace = FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.75;
    int thickness = 1;
    int baseline = 0;

    // Get the size of the text
    Size textSize = getTextSize(distanceStr, fontFace, fontScale, thickness, &baseline);
    baseline += thickness;

    // Use Rect values from faceDetect to set the text/box location
    Rect rectFace = m_faceROI;
    int boxBuffer = 15, boxMargin = 5;

    // These values are used to center the textBox horizontally with the faceRectangle
    int textCoordX = rectFace.x + ((rectFace.width - textSize.width) / 2);
    int textCoordY = rectFace.y + rectFace.height + textSize.height + boxBuffer;
    Point textPoint(textCoordX, textCoordY);

    // Draw the box
    rectangle(m_outputFrame, textPoint + Point(-boxMargin, boxMargin),
                textPoint + Point(textSize.width, -textSize.height) + Point(boxMargin, -boxMargin),
                Scalar::all(0), CV_FILLED);

    // Place text in the box
    putText(m_outputFrame, distanceStr, textPoint, fontFace, fontScale, Scalar::all(255), thickness, 8);
}

// Takes a vector of facial ROI's detected by the cascade, and returns the Rect belonging to the biggest one.
Rect FaceDetection::findLargestFaceROI(vector<Rect> rectFaces) {
    // default value, indicates no real faces were found
    Rect largestROI = Rect(0,0,10,10);
    if (rectFaces.empty()) {
        cout << "[findLargestFaceROI] No faces found." << endl;
        return largestROI;
    }
    largestROI = rectFaces[0];
    for (int i = 1; i < rectFaces.size(); i++) {
        if (rectFaces[i].size().area() > largestROI.size().area()) {
            largestROI = rectFaces[i];
        }
    }

    return largestROI;
}

bool FaceDetection::loadCascades() {
    // Load the cascade
    if (!face_cascade.load(face_cascade_name)) {
        cout << "[FaceDetection] Error loading face cascade." << endl;
        return false;
    } else {
        cout << "[FaceDetection] Successfully loaded cascade." << endl;
        Q = stereoCalibration->getQMatrix();
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