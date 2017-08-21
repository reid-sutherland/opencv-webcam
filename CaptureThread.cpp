#include "CaptureThread.h"
#include "Config.h"

using namespace std;

CaptureThread::CaptureThread(SharedImageBuffer *sharedImageBuffer, int deviceNumber, bool dropFrameIfBufferFull, int width, int height) :
    thread(&CaptureThread::run, this)  //Superclass (Thread.h) constructor, links thread with run()

{
    m_sharedImageBuffer = sharedImageBuffer;
    m_dropFrameIfBufferFull = dropFrameIfBufferFull;
    m_deviceNumber = deviceNumber;
    m_width = width;
    m_height = height;
    m_doStop = false;
    m_sampleNumber = 0;
    m_fpsSum = 0;
    m_fps = queue<int>();   //clear queue
    m_statsData.averageFPS = 0;
    m_statsData.nFramesProcessed = 0;

    m_start = false;
}

void CaptureThread::run()
{
    while (!m_start) {
        this_thread::yield();
    }
    while (1) {
        ////////////////////////////////
        // Stop thread if doStop=TRUE //
        ////////////////////////////////
        m_doStopMutex.lock();
        if (m_doStop) {
            m_doStop = false;
            m_doStopMutex.unlock();
            break;
        }
        m_doStopMutex.unlock();
        /////////////////////////////////
        /////////////////////////////////

        // Synchronize with other streams (if enabled for this stream)
        m_sharedImageBuffer->sync(m_deviceNumber);

        // Capture frame (if available)
        if (!m_cap.grab()) {
            continue;
        }

        // Retrieve frame
        m_cap.retrieve(m_grabbedFrame);

        // Add frame to buffer
        m_sharedImageBuffer->getByDeviceNumber(m_deviceNumber)->add(m_grabbedFrame, m_dropFrameIfBufferFull);
    }

    cerr << "Stopping capture thread..." << endl;
}

bool CaptureThread::connectToCamera()
{
    // Open camera
    bool camOpenResult = m_cap.open(m_deviceNumber);
    // Set resolution
    if (m_width != -1)
    {
        m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
    }
    if (m_height != -1)
    {
        m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
    }
    // Return result
    return camOpenResult;
}

bool CaptureThread::disconnectCamera()
{
    // Camera is connected
    if (m_cap.isOpened())
    {
        // Disconnect camera
        m_cap.release();
        return true;
    }
    // Camera is NOT connected
    else
    {
        return false;
    }
}

void CaptureThread::start()
{
    m_start = true;
}

void CaptureThread::stop()
{
    lock_guard<mutex> lck(m_doStopMutex);
    m_doStop = true;
}

bool CaptureThread::isCameraConnected()
{
    return m_cap.isOpened();
}