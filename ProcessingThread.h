#ifndef PROCESSINGTHREAD_H
#define PROCESSINGTHREAD_H

#include <thread>
#include <time.h>
#include <queue>
#include <mutex>
#include <string>

#include <opencv2/opencv.hpp>

#include "Structures.h"
#include "SharedImageBuffer.h"
class CameraView;

using namespace std;
using namespace cv;

class ProcessingThread : public thread
{
public:
    ProcessingThread(SharedImageBuffer *sharedImageBuffer, int deviceNumber, CameraView *parentCameraView);
    void stop();

private:
    void emitNewFrame(cv::Mat frame);

    SharedImageBuffer *m_sharedImageBuffer;
    CameraView *m_parentCameraView;
    cv::Mat m_currentFrame;
    time_t m_t;
    mutex m_doStopMutex;
    mutex m_processingMutex;
    ThreadStatisticsData m_statsData;
    volatile bool m_doStop;
    int m_deviceNumber;

protected:
    void run();
};

#endif // PROCESSINGTHREAD_H
