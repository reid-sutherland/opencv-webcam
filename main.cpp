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

//Global Variables for processing/3D reconstruction
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
Disparity disparityObject;
StereoCalibration stereoCalibration(StereoCalibration::Instance());
Reconstruction3D reconstruction3D(disparityObject);


bool connectCameras();
void calibrateCameras();
void processFeeds();
void objectDetectionImage(Mat input, Mat &contoured, bool fromPicture);
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
    //call XInitThreads
    XInitThreads();

    //print welcmome message
    welcome();

    //detect connected cameras
    props = util.detectCameras();

    //print menu options
    menuOptions();

    sharedImageBuffer = new SharedImageBuffer();
    char input = ' ';

    //TODO: remove this
    cout << "ObjectDetection (Image)?? (y/n)" << endl;
    if ((char) getch() == 'y') {
        Mat input, contoured;
        objectDetectionImage(input, contoured, true);
        return 0;
    }
    while (input != 'q') {
        if (props->getNumberOfCameras() <= 2 && props->getNumberOfCameras() > 0) {
            deviceIDs = props->getDeviceIDs();
            cout << "\n***NOTICE***: Only two cameras found, automatically starting camera connection." << endl;
            cout << "Connecting to cameras [" << deviceIDs[0] << "] and [" << deviceIDs[1] << "]" << endl << endl;

            connectCameras();

            cout << "\nAll cameras successfully disconnected." << endl;
            break;
        }
        if (props->getNumberOfCameras() == 0) {
            cout << "\n***NOTICE***: No cameras were found, please plug at least two in and try again." << endl << endl;
            break;
        }

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
            //if deviceIDs is empty, return user to menu loop
            if (deviceIDs.empty()) {
                cout << "Error: your list of devices is empty. Please add a device before attempting to connect." << endl;
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

    calibrateCameras();

    processFeeds();

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

void calibrateCameras() {
    MyCalibration mc;

    bool calibLoaded = false;
    char input;

    cout << "Do you want to calibrate cameras ? (y/n)" << endl;
    cout << "Note: otherwise a calibration file will be loaded.\n" << endl;
    while (!calibLoaded){
        input = (char) getch();

        //input = Y or y
        if (input == 'y' || input == 'Y') {
            mc.createCalibration(deviceIDs, cameraViewMap, stereoCalibration);
            calibLoaded = true;
        }
        else {
            //input = N or n
            if (input == 'n' || input == 'N') {
                stereoCalibration.setCalibFilename(CALIB_DEFAULT_FILENAME);
                if (stereoCalibration.loadCalib()) {
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
    Mat frame1;
    Mat frame2;
    Mat original1;
    Mat original2;
    Mat contoured;
    Mat imgDisparity32F;
    bool test = false;
    bool showRectified = false;
    bool showDisparity = true;
    char c;

    time_t startTime;
    time_t loopTime;
    time(&startTime);

    cout << "\n***Press d to attempt to detect objects from the current frame.***" << endl << endl;

    while (true) {

        cameraViewMap[deviceIDs[0]]->getFrame(frame1);
        cameraViewMap[deviceIDs[1]]->getFrame(frame2);

        //test
        if (test) {
            original1 = frame1.clone();
            original2 = frame2.clone();
        }

        // Rectify Images
        stereoCalibration.rectifyStereoImg(frame1, frame2, frame1, frame2);

        reconstruction3D.buildPointCloud(frame1, frame2, stereoCalibration);
        //disparityObject.m_semi_filtered_disp.convertTo(imgDisparity32F, CV_32F, 1./16);

        //reconstruction3D.retrievePointCloud(cloud);

        //PCObjectDetection objectDetection;
        //objectDetection.detectionPlaneSeg(cloud);
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_XYZ2(new pcl::PointCloud<pcl::PointXYZRGB>);
        //if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("/home/arthur/Documents/Internship/stereovis/applicationFiles/PlanarSeg/planarSeg.pcd", *cloud_XYZ2) != 0)
        //{
        //    return -1;
        //}

        //std::cout << "PCD File loaded sucessfully \n" << std::endl;

        //objectDetection.detectionEuclidianClustering(cloud_XYZ2);

        // Display windows
        if (test && !original1.empty() && !original2.empty()) {
            imshow("Original Left", original1);
            imshow("Original Right", original2);
        }
        if (showRectified && !frame1.empty() && !frame2.empty()) {
            imshow("Rectified Left", frame1);
            imshow("Rectified Right", frame2);
        }
        if (showDisparity && !disparityObject.filtered_disp.empty()) {
            imshow("Filtered Disparity", disparityObject.filtered_disp);
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