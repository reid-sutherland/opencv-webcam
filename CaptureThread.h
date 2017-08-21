#ifndef CAPTURETHREAD_H
#define CAPTURETHREAD_H

#include <thread>
#include <time.h>
#include <queue>
#include <mutex>
#include <string>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include "SharedImageBuffer.h"
#include "Structures.h"
class CameraView;

using namespace std;

class CaptureThread : public thread
{
public:
    CaptureThread(SharedImageBuffer *sharedImageBuffer, int deviceNumber, bool dropFrameIfBufferFull, int width, int height);
    void start();
    void stop();
    bool connectToCamera();
    bool disconnectCamera();
    bool isCameraConnected();

private:
    SharedImageBuffer *m_sharedImageBuffer;
    cv::VideoCapture m_cap;
    cv::Mat m_grabbedFrame;
    mutex m_doStopMutex;
    queue<int> m_fps;
    ThreadStatisticsData m_statsData;
    volatile bool m_doStop;
    int m_captureTime;
    int m_sampleNumber;
    int m_fpsSum;
    bool m_dropFrameIfBufferFull;
    int m_deviceNumber;
    int m_width;
    int m_height;

    bool m_start;

protected:
    void run();
};

#endif // CAPTURETHREAD_H
