#include "ProcessingThread.h"
#include "CameraView.h"
#include "Config.h"

using namespace std;

ProcessingThread::ProcessingThread(SharedImageBuffer *sharedImageBuffer, int deviceNumber, CameraView *parentCameraView) :
        thread(&ProcessingThread::run, this)
{
    m_sharedImageBuffer = sharedImageBuffer;
    m_deviceNumber = deviceNumber;
    m_parentCameraView = parentCameraView;
    m_doStop = false;
    m_statsData.averageFPS = 0;
    m_statsData.nFramesProcessed = 0;
}

void ProcessingThread::run()
{
    int frameCount = 1;
    int frameCountDegree = 1;
    int rectVertexTwoX = 16;

    while(1)
    {
        ////////////////////////////////
        // Stop thread if doStop=TRUE //
        ////////////////////////////////
        m_doStopMutex.lock();
        if (m_doStop)
        {
            m_doStop = false;
            m_doStopMutex.unlock();
            break;
        }
        m_doStopMutex.unlock();
        /////////////////////////////////
        /////////////////////////////////

        m_processingMutex.lock();

        // Get frame from queue, store in currentFrame
        if (m_sharedImageBuffer->getByDeviceNumber(m_deviceNumber) != nullptr)
        {
            m_currentFrame = cv::Mat(m_sharedImageBuffer->getByDeviceNumber(m_deviceNumber)->get().clone());
        }
        else {
            cout << "getByDeviceNumber == NULL" << endl;
        }

        //Display frame count
        /*
        if (frameCount == (pow(10, frameCountDegree))) {
            frameCountDegree++;         //increments every time a digit is added to frameCount (i.e. 100, 1000, etc.)
            rectVertexTwoX += 10;       //extends the whitespace rectangle to fit the new frameCount (which is now one digit bigger)
        }
        rectangle(m_currentFrame, cv::Point(0, 0), cv::Point(rectVertexTwoX, 16), cv::Scalar(255, 255, 255), -1);
        putText(m_currentFrame, to_string(frameCount), cv::Point(3, 13), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
        */

        /*
        //Display image buffer bar (% of buffer full)
        string imageBufferString;
        imageBufferString.append(to_string(m_sharedImageBuffer->getByDeviceNumber(m_deviceNumber)->size()
                                 / m_sharedImageBuffer->getByDeviceNumber(m_deviceNumber)->maxSize()));
        imageBufferString.append("% full");
        rectangle(m_currentFrame, cv::Point(0, 18), cv::Point(58, 36), cv::Scalar(255, 255, 255), -1);
        putText(m_currentFrame, imageBufferString, cv::Point(3, 31), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
        */

        m_processingMutex.unlock();

        // Inform GUI thread of new frame
        emitNewFrame(m_currentFrame);

        //increment frameCount
        frameCount++;
    }

    cerr << "Stopping processing thread..." << endl;
}

void ProcessingThread::emitNewFrame(Mat frame) {
    m_parentCameraView->updateFrame(frame);
}

void ProcessingThread::stop()
{
    lock_guard<mutex> lck(m_doStopMutex);
    m_doStop = true;

}

/*
////////////////////////////////////
// PERFORM IMAGE PROCESSING BELOW //
////////////////////////////////////
// Grayscale conversion
if (m_imgProcFlags.grayscaleOn && (m_currentFrame.channels() == 3 || m_currentFrame.channels() == 4))
{
    cvtColor(m_currentFrame,
             m_currentFrame,
             CV_BGR2GRAY);
}

// Smooth
if (m_imgProcFlags.smoothOn)
{
    switch (m_imgProcSettings.smoothType)
    {
        // Blur
        case 0:
            blur(m_currentFrame,
                 m_currentFrame,
                 cv::Size(m_imgProcSettings.smoothParam1, m_imgProcSettings.smoothParam2));
            break;
            // Gaussian
        case 1:
            GaussianBlur(m_currentFrame,
                         m_currentFrame,
                         cv::Size(m_imgProcSettings.smoothParam1, m_imgProcSettings.smoothParam2),
                         m_imgProcSettings.smoothParam3,
                         m_imgProcSettings.smoothParam4);
            break;
            // Median
        case 2:
            medianBlur(m_currentFrame,
                       m_currentFrame,
                       m_imgProcSettings.smoothParam1);
            break;
    }
}
// Dilate
if (m_imgProcFlags.dilateOn)
{
    dilate(m_currentFrame,
           m_currentFrame,
           cv::Mat(),
           cv::Point(-1, -1),
           m_imgProcSettings.dilateNumberOfIterations);
}
// Erode
if (m_imgProcFlags.erodeOn)
{
    erode(m_currentFrame,
          m_currentFrame,
          cv::Mat(),
          cv::Point(-1, -1),
          m_imgProcSettings.erodeNumberOfIterations);
}
// Flip
if (m_imgProcFlags.flipOn)
{
    flip(m_currentFrame,
         m_currentFrame,
         m_imgProcSettings.flipCode);
}
// Canny edge detection
if (m_imgProcFlags.cannyOn)
{
    Canny(m_currentFrame,
          m_currentFrame,
          m_imgProcSettings.cannyThreshold1,
          m_imgProcSettings.cannyThreshold2,
          m_imgProcSettings.cannyApertureSize,
          m_imgProcSettings.cannyL2gradient);
}
////////////////////////////////////
// PERFORM IMAGE PROCESSING ABOVE //
////////////////////////////////////
*/