//#define STAT

//Opencv
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"

//PCL
#include <pcl/PCLPointCloud2.h>

//Local
#include "UtilsCameras.h"
#include "CameraView.h"
#include "StereoCalibration.h"
#include "MyCalibration.h"
#include "FilenameManagement.h"
#include "Reconstruction3D.h"
#include "PCObjectDetection.h"
#include "FaceDetection.h"
//#include "Disparity.h"

//System
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <termios.h>
#include <atomic>
#include <X11/Xlib.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>

using namespace std;
using namespace cv;

//Global Variables
CameraProperties *props;
UtilsCameras util;
SharedImageBuffer *sharedImageBuffer;
vector<int> deviceIDs;
map<int, CameraView*> cameraViewMap;
mutex delete_mutex;
bool isVR;

//Global Variables for processing/3D reconstruction
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
Disparity disparityObject;
StereoCalibration* stereoCalibration(StereoCalibration::instance());
Reconstruction3D reconstruction3D(disparityObject);
FaceDetection faceDetection(disparityObject);
Mat Q;


bool connectCameras();
void assignCameras();
void calibrateCameras();
void processFeeds();
void staticROIMethod();
void faceDetectionMethod();
void faceTrackingMethod();
void backgroundSubtractionMethod();
void objectDetectionImage(Mat input, Mat &contoured, bool fromPicture);
double distanceCalculator(Rect2d ROI, Mat &outputFrame);
void drawDistanceBelowROI(Rect2d ROI, double distance, Mat &outputFrame);
int getch();

void welcome() {
    cout
        << endl
        << "****************Welcome to Stereo Vision!*****************" << endl
        << "opencv-webcam"												<< endl
        << endl;
}

void menuOptions() {
    cout
        << endl
        << "---------------------------------------------------------------------------"                << endl
        << "Menu Options:"                                                                              << endl
                                                                                                        << endl
        << "To add devices to the list of devices to be displayed, press the number key"                << endl
        << "\tassociated with the device ID of the device you would like to add."                       << endl
        << "\tExample: Enter \"0\" to connect to dev/video/0."                                          << endl
        << "Press r to clear your list and start over."                                                 << endl
        << "Press l to list the devices you have already selected."                                     << endl
        << "Press c to attempt to connect to your list of devices."                                     << endl
        << "Press i to refresh and display the list of found devices."                                  << endl
        << "Press h to display this help message again."                                                << endl
        << "Press q to quit."                                                                           << endl
        << "---------------------------------------------------------------------------"                << endl;
}

int main (int argc, char* argv[])
{
    // Call XInitThreads
    XInitThreads();

    // Print welcmome message
    welcome();

    // Detect connected cameras
    props = util.detectCameras();

    // Print help message
    cout << "\n------------------------------------------" << endl;
    cout << "Press h to display a list of menu options." << endl;
    cout << "------------------------------------------" << endl;

    sharedImageBuffer = new SharedImageBuffer();
    char input = ' ';

    if (props->getNumberOfCameras() == 2) {
        deviceIDs = props->getDeviceIDs();
        cout << "\n***NOTICE***: Only two cameras found, automatically starting camera connection." << endl;
        cout << "Connecting to cameras [" << deviceIDs[0] << "] and [" << deviceIDs[1] << "]" << endl << endl;

        input = 'c';
    }
    if (props->getNumberOfCameras() == 0) {
        cout << "\n***NOTICE***: No cameras were found, please plug at least two in and try again." << endl << endl;

        input = 'q';
    }

    while (input != 'q') {

        cout << "\n*****Please select an option.*****" << endl << endl;
        input = (char) getch();

        //input is an integer
        if (input >= 48 && input <= 57) {
            //set id equal to input's integer value
            int id = (int) input - 48;

            //check for duplicates
            bool duplicate = false;
            for (auto& deviceID : deviceIDs) {
                if (id == deviceID) {
                    duplicate = true;
                    break;
                }
            }

            if (duplicate) {
                cout << "Error: a device with ID \"" << id << "\" is already in your list."
                     << " (" << props->getDeviceNameByID(id) << ")" << endl;
            }
            else if (deviceIDs.size() > 2) {
                cout << "Error: No more than 2 cameras may be selected." << endl;
            }
            else if (props->containsDeviceID(id)) {
                deviceIDs.push_back(id);
                cout << "Success! " << props->getDeviceNameByID(id) << " was successfully added." << endl;
            }
            else {
                cerr << "Error: ID \"" << id << "\" does not match any known devices. Please try again." << endl;
            }
        }

        //remove device
        else if (input == 'r') {
            cout << "Please enter the ID of the device that you would like to remove." << endl;
            int id = getch() - 48;
            bool match = false;

            //loop through devices
            for (int i = 0; i < deviceIDs.size(); i++) {
                if (id == deviceIDs[i]) {
                    deviceIDs.erase(deviceIDs.begin() + i);     //erase the matched element
                    match = true;
                    cout << "Success! " << props->getDeviceNameByID(id) << " was successfully removed." << endl;
                    break;
                }
            }

            if (!match) {
                cerr << "Error: ID \"" << id << "\" does not match any devices in your list." << endl;
            }
        }

        //list devices
        else if (input == 'l') {
            if (deviceIDs.empty()) {
                cout << "Your list is currently empty." << endl;
            }
            else {
                cout << "Your current list of added devices:" << endl;
                //loop through deviceIDs
                for (auto &id : deviceIDs) {
                    //display name of each device
                    cout << props->getDeviceNameByID(id) << endl;
                }
            }
        }

        //attempt to connect to devices
        else if (input == 'c') {
            // if deviceIDs is empty, return user to menu loop
            if (deviceIDs.empty()) {
                cout << "Error: your list of devices is empty. Please add a device before attempting to connect." << endl;
                continue;
            }
            else if (deviceIDs.size() < 2) {
                cout << "Error: you must connect to exactly 2 cameras. Please add 2 cameras before attempting to connect." << endl;
                continue;
            }

            connectCameras();

            cout << "\nAll cameras successfully disconnected." << endl;
            break;
        }

        //refresh/display found devices
        else if (input == 'i') {
            props = util.detectCameras();
        }

        //print help message
        else if (input == 'h') {
            menuOptions();
        }

        //quit
        else if (input == 'q') {
        }

        //incorrect input
        else {
            cerr << "Error: That is not a valid option. Please try again." << endl;
        }
    }

    delete sharedImageBuffer;
    delete props;

    exit(0);
}

// This function initializes the camera connection process, and creates all of the capturing/processing threads
bool connectCameras() {
    //always enable stream synchronization
    bool streamSync = true;
    int defaultBufferSize = 100;
    bool defaultEnableFrameDrop = true;

    //for each camera registered in deviceIDs...
    for (auto deviceID : deviceIDs) {
        //create a new imageBuffer
        auto *imageBuffer = new Buffer<Mat>(defaultBufferSize);

        //add each imageBuffer to the sharedImageBuffer
        sharedImageBuffer->add(deviceID, imageBuffer, streamSync);

        //create cameraView
        cameraViewMap[deviceID] = new CameraView(deviceID, sharedImageBuffer);
    }
    sharedImageBuffer->setSyncEnabled(streamSync);

    //attempt to connect to each camera
    for (auto deviceID : deviceIDs) {
        if (cameraViewMap[deviceID]->connectToCamera(defaultEnableFrameDrop, -1, -1)) {
            cerr << "[ " << deviceID << " ] Connection Successful!\n" << endl;
        }
        else {
            cerr << "[ " << deviceID << " ] Connection Unsuccessful..." << endl;
            return false;
        }
    }

    // If cameras are VR, calibration works a bit differently
    cout << "*** VR Camera? (y/n) ***" << endl;
    char c = (char) getch();
    if (c == 'y' || c == 'Y') {
        // Swap camera indices if VR Camera is being used
        int temp = deviceIDs[0];
        deviceIDs[0] = deviceIDs[1];
        deviceIDs[1] = temp;

        // Set flag
        isVR = true;
    } else { isVR = false; }

    /*
    // For some cameras, we must specify which lens is the left and which lens is the right
    cout << "*** Check camera orientation? (y/n) ***" << endl;
    char c = (char) getch();
    if (c == 'y' || c == 'Y') {
        assignCameras();
    } else { cout << endl; }    // formatting
     */

    calibrateCameras();
    Q = stereoCalibration->getQMatrix();

    cout << "***Method Selection***" << endl;
    cout << "Press r for 3D Reconstruction, f for face detection, t for face tracking, "
         << "b for background subtraction, or s for static ROI." << endl << endl;

    c = (char) getch();

    stereoCalibration->printQMatrix();

    if (c == 'b' || c == 'B') {
        backgroundSubtractionMethod();
    } else if (c == 's' || c == 'S') {
        staticROIMethod();
    } else if (c == 'f' || c == 'F') {
        faceDetectionMethod();
    } else if (c == 't' || c == 'T') {
        faceTrackingMethod();
    } else {
        processFeeds();
    }

    //after disconnection
    for (auto deviceID : deviceIDs) {
        delete_mutex.lock();

        //Explicitly delete cameraView
        delete cameraViewMap[deviceID];
        cameraViewMap.erase(deviceID);

        delete_mutex.unlock();
    }

    return true;
}

// This function allows the user to test the cameras and make sure that both cameras are in the correct orientation
// If the user determines there is a problem, they can choose to swap the camera IDs
void assignCameras() {
    Mat frame1, frame2;

    cout << "Press Esc once you have determined if the camera orientation is correct." << endl << endl;

    while (true) {
        cameraViewMap[deviceIDs[0]]->getFrame(frame1);
        cameraViewMap[deviceIDs[1]]->getFrame(frame2);

        if (!frame1.empty() && !frame2.empty()) {
            imshow("Left Camera", frame1);
            imshow("Right Camera", frame2);
        }

        // Esc
        if (waitKey(50) == 27) {
            destroyAllWindows();
            break;
        }
    }

    cout << "Swap cameras? (y/n)" << endl;
    auto c = (char) getch();

    if (c == 'y' || c == 'Y') {
        // Swap camera IDs
        int temp = deviceIDs[0];
        deviceIDs[0] = deviceIDs[1];
        deviceIDs[1] = temp;

        cout << "Cameras swapped!\n" << endl;
    }
    else {
        // do not swap
        cout << "Cameras NOT swapped.\n" << endl;
    }
}

void calibrateCameras() {
    MyCalibration mc;

    bool calibLoaded = false;
    char input;

    cout << "Do you want to calibrate cameras? (y/n)" << endl;
    cout << "Note: otherwise a calibration file will be loaded.\n" << endl;
    while (!calibLoaded){
        input = (char) getch();

        // Yes
        if (input == 'y' || input == 'Y') {
            if (isVR) {
                mc.createVRCalibration(deviceIDs, cameraViewMap);
            } else {
                mc.createCalibration(deviceIDs, cameraViewMap);
            }
            calibLoaded = true;
        }
        // No
        else if (input == 'n' || input == 'N') {
            if (stereoCalibration->loadCalib(isVR)) {
                calibLoaded = true;
            }
            else {
                cout << "\nDo you want to calibrate cameras ? (y/n)" << endl;
                cout << "Note: otherwise a calibration file will be loaded.\n" << endl;
            }
        }
        else {
            cout << "\nError: Please select yes [y] or no [n]." << endl;
        }
        }
    }
}

void processFeeds() {
    Mat frame1, frame2, original1, original2, contoured;
    Mat disparity;
    bool test = false;
    bool showRectified = true;
    bool showDisparity = true;
    char c;

    time_t startTime;
    time_t loopTime;
    time(&startTime);

    cout << "\n***Press d to attempt to detect objects from the current frame.***" << endl << endl;

    while (true) {

        cameraViewMap[deviceIDs[0]]->getFrame(frame1);
        cameraViewMap[deviceIDs[1]]->getFrame(frame2);

        if (test && !frame1.empty() && !frame2.empty()) {
            original1 = frame1.clone();
            original2 = frame2.clone();
        }

        // Rectify Images
        stereoCalibration->rectifyStereoImg(frame1, frame2, frame1, frame2);

        // Compute Disparity Map
        disparityObject.computeDispMap(frame1, frame2);
        disparity = disparityObject.filtered_disp;

        if (test && !frame1.empty() && !frame2.empty()) {
            imshow("Original Left", frame1);
            imshow("Original Right", frame2);
        }
        if (showRectified && !frame1.empty() && !frame2.empty()) {
            imshow("Rectified Left", frame1);
            imshow("Rectified Right", frame2);
        }
        if (showDisparity && !disparity.empty()) {
            imshow("Filtered Disparity", disparity);
        }

        c = (char) waitKey(50);
        if ((int) c == 27) {
            destroyAllWindows();
            break;
        }
        else if (c == 'd') {
            objectDetectionImage(frame1, contoured, false);
            imshow("Contoured", contoured);
        }

#ifdef STAT
        count++;
        cout << "Number of iterations: " << count << endl;
        time(&loopTime);
        cout << "Timer = " << difftime(loopTime, startTime) << " seconds." << endl << endl;
#endif
    }
}

void staticROIMethod() {
    Mat frame1, frame2, outputFrame;
    Rect2d ROI;
    char c;
    double distance;

    cameraViewMap[deviceIDs[0]]->getFrame(outputFrame);

    ROI = Rect2d(outputFrame.cols*3/8, outputFrame.rows*3/8, outputFrame.cols/4, outputFrame.rows/4);

    cout << "***Center the object in the ROI to calculate its distance.***" << endl << endl;

    while (true) {

        cameraViewMap[deviceIDs[0]]->getFrame(frame1);
        cameraViewMap[deviceIDs[1]]->getFrame(frame2);

        // Rectify Images
        stereoCalibration->rectifyStereoImg(frame1, frame2, frame1, frame2);

        // Compute Disparity Map
        disparityObject.computeDispMap(frame1, frame2);

        // Initialize outputFrame
        outputFrame = frame1.clone();

        // Draw ROI on frame
        Point2d P1(ROI.x, ROI.y);
        Point2d P2(ROI.x + ROI.width, ROI.y + ROI.height);
        rectangle(outputFrame, P1, P2, Scalar(255, 0, 0), 1, 8, 0);

        // Calculate Distance
        distance = distanceCalculator(ROI, outputFrame);

        // Draw Distance Below ROI
        drawDistanceBelowROI(ROI, distance, outputFrame);

        // Show outputFrame
        imshow("Static ROI", outputFrame);

        c = (char) waitKey(50);
        if ((int) c == 27) {
            destroyAllWindows();
            break;
        }
    }
}

void faceDetectionMethod() {
    Mat frame1, frame2, distanceFrame;
    vector<Rect> faces;
    vector<Point> centers;
    bool showFaceDetection = true;
    char c;

    // Load the cascade(s)
    if (!faceDetection.loadCascades()) {
        return;
    }
    faceDetection.updateQMatrix();

    while (true) {
        cameraViewMap[deviceIDs[0]]->getFrame(frame1);
        cameraViewMap[deviceIDs[1]]->getFrame(frame2);

        // Rectify Images
        stereoCalibration->rectifyStereoImg(frame1, frame2, frame1, frame2);

        // Compute Disparity Map
        //this can be done directly in computeDistanceToFace()
        //disparityObject.computeDispMap(frame1, frame2);

        // Detect Faces
        faceDetection.detectFace(frame1, frame2);

        // Compute Distance to Face
        faceDetection.computeDistanceToFace(true);

        // Draw the distance on the image
        faceDetection.drawDistanceBelowROI(false);  // false is used for face detection

        // Copy outputFrame to distanceFrame
        faceDetection.outputFrame().copyTo(distanceFrame);

        if (showFaceDetection && !distanceFrame.empty()) {
            imshow("FaceDetection", distanceFrame);
        }

        c = (char) waitKey(50);
        if ((int) c == 27) {
            destroyAllWindows();
            break;
        }
    }
}

void faceTrackingMethod() {
    Mat frame1, frame2, distanceFrame;
    bool showTrackedObject = true;
    char c;

    cout << "***Press c when you are ready to capture an image from the left camera, "  << endl
         << "\twhich you will use to select an ROI for the object to be tracked."       << endl
         << "\tBe sure the object to be tracked is clearly displayed in the image.***"  << endl << endl;

    while (c != 27) {
        cameraViewMap[deviceIDs[0]]->getFrame(frame1);
        cameraViewMap[deviceIDs[1]]->getFrame(frame2);

        // Rectify Images
        stereoCalibration->rectifyStereoImg(frame1, frame2, frame1, frame2);

        imshow("Press c to capture frame", frame1);

        c = (char) waitKey(50);
        if (c == 'c' || c == 'C') {
            if (faceDetection.initializeTrackerROI(frame1)) {   // Valid ROI
                destroyAllWindows();
                break;
            } else {    // Invalid ROI
                continue;
            }
        }
    }

    cout << "\n***Starting the tracking process, press ESC to quit." << endl << endl;
    faceDetection.updateQMatrix();

    while (true) {
        cameraViewMap[deviceIDs[0]]->getFrame(frame1);
        cameraViewMap[deviceIDs[1]]->getFrame(frame2);

        // Rectify Images
        stereoCalibration->rectifyStereoImg(frame1, frame2, frame1, frame2);

        // Compute Disparity Map
        //this can be done directly in computeDistanceToFace()
        //disparityObject.computeDispMap(frame1, frame2);

        // Track Face/Object
        faceDetection.trackFace(frame1, frame2);

        // Compute Distance to Face
        faceDetection.computeDistanceToObject(true);

        // Draw the distance on the image
        faceDetection.drawDistanceInCorner();

        // Copy outputFrame to distanceFrame
        faceDetection.outputFrame().copyTo(distanceFrame);

        if (showTrackedObject && !distanceFrame.empty()) {
            imshow("Tracking", distanceFrame);
        }

        c = (char) waitKey(50);
        if ((int) c == 27) {
            destroyAllWindows();
            break;
        }
    }
}

void backgroundSubtractionMethod() {
    Mat frame1, frame2, fgMask1, fgMask2;
    Mat contoured;
    Ptr<BackgroundSubtractor> pBS1 = createBackgroundSubtractorMOG2();
    Ptr<BackgroundSubtractor> pBS2 = createBackgroundSubtractorMOG2();
    char c;

    while (true) {

        cameraViewMap[deviceIDs[0]]->getFrame(frame1);
        cameraViewMap[deviceIDs[1]]->getFrame(frame2);

        // Rectify Images
        stereoCalibration->rectifyStereoImg(frame1, frame2, frame1, frame2);

        // Update background models
        pBS1->apply(frame1, fgMask1);
        pBS2->apply(frame2, fgMask2);

        // Show current frames
        imshow("FG Mask Left", fgMask1);
        imshow("FG Mask Right", fgMask2);

        c = (char) waitKey(50);
        if ((int) c == 27) {
            destroyAllWindows();
            break;
        }
    }
}

void objectDetectionImage(Mat input, Mat &contoured, bool fromPicture) {
    // load the color image
    IplImage* im;
    if (fromPicture) {
        im = cvLoadImage("/home/reid/Pictures/objects.jpg");
    }
    else {
        im = cvCreateImage(cvSize(input.cols, input.rows), 8, 3);
        IplImage temp = input;
        cvCopy(&temp, im);
    }

    // get the color histogram
    IplImage* im32f = cvCreateImage(cvGetSize(im), IPL_DEPTH_32F, 3);
    cvConvertScale(im, im32f);

    int channels[] = {0, 1, 2};
    int histSize[] = {32, 32, 32};
    float rgbRange[] = {0, 256};
    float* ranges[] = {rgbRange, rgbRange, rgbRange};

    CvHistogram* hist = cvCreateHist(3, histSize, CV_HIST_ARRAY, ranges);
    IplImage* b = cvCreateImage(cvGetSize(im32f), IPL_DEPTH_32F, 1);
    IplImage* g = cvCreateImage(cvGetSize(im32f), IPL_DEPTH_32F, 1);
    IplImage* r = cvCreateImage(cvGetSize(im32f), IPL_DEPTH_32F, 1);
    IplImage* backproject32f = cvCreateImage(cvGetSize(im), IPL_DEPTH_32F, 1);
    IplImage* backproject8u = cvCreateImage(cvGetSize(im), IPL_DEPTH_8U, 1);
    IplImage* bw = cvCreateImage(cvGetSize(im), IPL_DEPTH_8U, 1);
    IplConvKernel* kernel = cvCreateStructuringElementEx(3, 3, 1, 1, MORPH_ELLIPSE);

    cvSplit(im32f, b, g, r, nullptr);
    IplImage* planes[] = {b, g, r};
    cvCalcHist(planes, hist);

    // find min and max values of histogram bins
    float minval, maxval;
    cvGetMinMaxHistValue(hist, &minval, &maxval);

    // threshold the histogram. this sets the bin values that are below the threshold to zero
    cvThreshHist(hist, maxval/32);

    // backproject the thresholded histogram. backprojection should contain higher values for the
    // background and lower values for the foreground
    cvCalcBackProject(planes, backproject32f, hist);

    // convert to 8u type
    double min, max;
    cvMinMaxLoc(backproject32f, &min, &max);
    cvConvertScale(backproject32f, backproject8u, 255.0 / max);

    // threshold backprojected image. this gives us the background
    cvThreshold(backproject8u, bw, 10, 255, CV_THRESH_BINARY);

    // some morphology on background
    cvDilate(bw, bw, kernel, 1);
    cvMorphologyEx(bw, bw, nullptr, kernel, MORPH_CLOSE, 2);

    // get the foreground
    cvSubRS(bw, cvScalar(255, 255, 255), bw);
    cvMorphologyEx(bw, bw, nullptr, kernel, MORPH_OPEN, 2);
    cvErode(bw, bw, kernel, 1);

    // find contours of the foreground
    //CvMemStorage* storage = cvCreateMemStorage(0);
    //CvSeq* contours = 0;
    //cvFindContours(bw, storage, &contours);
    //cvDrawContours(im, contours, CV_RGB(255, 0, 0), CV_RGB(0, 0, 255), 1, 2);

    // grabcut
    Mat color = cvarrToMat(im);
    Mat fg = cvarrToMat(bw);
    Mat mask(bw->height, bw->width, CV_8U);

    mask.setTo(GC_PR_BGD);
    mask.setTo(GC_PR_FGD, fg);

    Mat bgModel, fgModel;
    grabCut(color, mask, Rect(), bgModel, fgModel, GC_INIT_WITH_MASK);
    Mat gcfg = mask == GC_PR_FGD;

    ///*
    int c = 0;
    while (c != 27) {
        imshow("gcfg", gcfg);
        c = waitKey(50);
    }
    destroyWindow("gcfg");
    //*/

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(gcfg, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    for(int idx = 0; idx < contours.size(); idx++)
    {
        Rect roi = boundingRect(contours[idx]);

        ///*
        if(roi.height>300 && roi.width>300) {
            cout << "idx: " << idx << " big" << endl;
            cout << roi.height << endl;     //***
            cout << roi.width << endl;
            continue;
        }

        if(roi.height<40 && roi.width<40) {
            cout << "idx: " << idx << " small" << endl;
            cout << roi.height << endl;     //***
            cout << roi.width << endl;
            continue;
        }
        //*/

        cout << "idx: " << idx << " SUCCESS" << endl;     //***
        cout << roi.height << endl;     //***
        cout << roi.width << endl;      //***

        drawContours(color, contours, idx, Scalar(0, 0, 255), 2);

        rectangle(color, roi,(255,0,255), 2);
    }

    contoured = color.clone();

    imwrite("/home/reid/Pictures/contoured.jpg", color) ;
}

// Distance Calculator
double distanceCalculator(Rect2d ROI, Mat &outputFrame) {

    // Calculate smallROI
    Rect2d smallROI = Rect2d(ROI.x + ROI.width/8,
                             ROI.y + ROI.height/8,
                             ROI.width*3/4,
                             ROI.height*3/4);

    Mat dispObjectFrame = Mat(disparityObject.filtered_disp, smallROI);

    // Find the average disparity across the ROI
    int sum = 0, d_int = 0, numValues = 0;
    for (int i = 0; i < dispObjectFrame.rows; i++) {
        auto *disp_row_ptr = dispObjectFrame.ptr<uchar>(i);

        for (int j = 0; j < dispObjectFrame.cols; j++) {
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

    double distance = (baseline * focal) / avgDisp;

#ifdef DEBUG
    stereoCalibration->printQMatrix();
    cout << "avgDisp = " << avgDisp << endl;
    cout << "baseline = " << baseline << endl;
    cout << "focal = " << focal << endl;
    cout << "***Distance to object is: " << m_distance << " meters***" << endl << endl;
#endif

    return distance;
}

void drawDistanceBelowROI(Rect2d ROI, double distance, Mat &outputFrame) {
    // Create the distance string
    string distanceStr = "Distance: ";
    distanceStr.append(to_string(distance));
    distanceStr.append(" (m)");

    // Initialize textDraw values
    int fontFace = FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.75;
    int thickness = 1;
    int baseline = 0;

    // Get the size of the text
    Size textSize = getTextSize(distanceStr, fontFace, fontScale, thickness, &baseline);
    baseline += thickness;

    int boxBuffer = 15, boxMargin = 5;

    // These values are used to center the textBox horizontally with the faceRectangle
    double textCoordX = ROI.x + ((ROI.width - textSize.width) / 2);
    double textCoordY = ROI.y + ROI.height + textSize.height + boxBuffer;
    Point2d textPoint(textCoordX, textCoordY);

    // Draw the box
    rectangle(outputFrame, textPoint + Point2d(-boxMargin, boxMargin),
              textPoint + Point2d(textSize.width, -textSize.height) + Point2d(boxMargin, -boxMargin),
              Scalar::all(0), CV_FILLED);

    // Place text in the box
    putText(outputFrame, distanceStr, textPoint, fontFace, fontScale, Scalar::all(255), thickness, 8);
}

//reads from keypress, doesn't echo
int getch()
{
    struct termios oldattr, newattr;
    int ch;
    tcgetattr( STDIN_FILENO, &oldattr );
    newattr = oldattr;
    newattr.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newattr );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldattr );
    return ch;
}