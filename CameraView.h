//
// Created by reid on 7/31/17.
//

#ifndef CAMERAVIEW_H
#define CAMERAVIEW_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/video.hpp>
#include "opencv2/videoio.hpp"
//#include <source/include/opencv/cv.h>

#include <iostream>
#include <string>
#include "Structures.h"
#include "SharedImageBuffer.h"
#include "CaptureThread.h"
#include "ProcessingThread.h"

using namespace std;
using namespace cv;

class CameraView {
public:
    explicit CameraView(int deviceNumber, SharedImageBuffer *sharedImageBuffer);
    ~CameraView();
    bool connectToCamera(bool dropFrame, int width, int height);
    void updateFrame(Mat &frame);
    bool getFrame(Mat &frame);
    bool esc();

private:
    void stopCaptureThread();
    void stopProcessingThread();

    int m_deviceNumber;
    bool m_isCameraConnected;
    static bool m_esc;
    string m_windowName;
    Mat m_currentFrame;

    CaptureThread *m_captureThread;
    SharedImageBuffer *m_sharedImageBuffer;
    ProcessingThread *m_processingThread;
};


#endif //CAMERAVIEW_H
