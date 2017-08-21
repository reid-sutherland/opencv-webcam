//
// Created by reid on 7/31/17.
//

#include "CameraView.h"

bool CameraView::m_esc = false;

CameraView::CameraView(int deviceNumber, SharedImageBuffer *sharedImageBuffer) :
    m_sharedImageBuffer(sharedImageBuffer),
    m_deviceNumber(deviceNumber)
{
}

CameraView::~CameraView() {
    if (m_isCameraConnected) {

        //Stop processing thread
        if (m_processingThread->joinable()) {
            stopProcessingThread();
        }

        //Stop capture thread
        if (m_captureThread->joinable()) {
            stopCaptureThread();
        }

        //Automatically start frame processing (for other streams)
        if (m_sharedImageBuffer->isSyncEnabledForDeviceNumber(m_deviceNumber)) {
            m_sharedImageBuffer->setSyncEnabled(true);
        }

        //Remove from shared buffer
        m_sharedImageBuffer->removeByDeviceNumber(m_deviceNumber);

        //destroyWindow(m_windowName);

        //Disconnect Camera
        if (m_captureThread->disconnectCamera()) {

            cerr << "[ " << m_deviceNumber << " ] Camera successfully disconnected.\n" << endl;
        }
        else {
            cerr << "[ " << m_deviceNumber << " ] WARNING: Camera already disconnected." << endl;
        }
    }
}

bool CameraView::connectToCamera(bool dropFrameIfBufferFull, int width, int height) {
    //Create Window Name - Add deviceNumber to the end of the generic window name
    m_windowName = "Capture feed - CAMERA ";
    m_windowName.append(to_string(m_deviceNumber));

    //Create Capture Thread
    m_captureThread = new CaptureThread(m_sharedImageBuffer, m_deviceNumber, dropFrameIfBufferFull, width, height);

    //Attempt to connect to camera
    if (m_captureThread->connectToCamera()) {

        //Wait for capture thread to connect to camera before letting the thread continue
        while (!m_captureThread->isCameraConnected()) {
            this_thread::yield();
            cout << "deviceID " << m_deviceNumber << " yielding..." << endl;
        }

        //start capture thread
        m_captureThread->start();

        //Create Window GUI for feed
        //namedWindow(m_windowName, WINDOW_AUTOSIZE);

        //Start processing captured frames
        m_processingThread = new ProcessingThread(m_sharedImageBuffer, m_deviceNumber, this);

        //Set internal flag and return
        m_isCameraConnected = true;

        return true;
    }
    //Failed to connect to camera
    else {
        return false;
    }
}

void CameraView::stopCaptureThread() {
    cerr << "[ " << m_deviceNumber << " ] About to stop capture thread..." << endl;
    m_captureThread->stop();
    m_sharedImageBuffer->notifyAll();     //This allows the thread to be stopped if it is in a wait-state

    //Take one frame off of a FULL queue to allow the capture thread to finish
    if (m_sharedImageBuffer->getByDeviceNumber(m_deviceNumber)->isFull()) {
        m_sharedImageBuffer->getByDeviceNumber(m_deviceNumber)->get();
    }

    //Wait for captureThread to finish its execution
    m_captureThread->join();

    cerr << "[ " << m_deviceNumber << " ] Capture thread successfully stopped." << endl;
}

void CameraView::stopProcessingThread() {
    cerr << "[ " << m_deviceNumber << " ] About to stop processing thread..." << endl;
    m_processingThread->stop();
    m_sharedImageBuffer->notifyAll();     //This allows the thread to be stopped if it is in a wait-state

    //Wait for processingThread to finish its execution
    m_processingThread->join();

    cerr << "[ " << m_deviceNumber << " ] Processing thread successfully stopped." << endl;
}

void CameraView::updateFrame(Mat &frame) {
    //Display frame from ProcessingThread
    if (!m_esc) {
        //imshow(m_windowName, frame);
    }
    frame.copyTo(m_currentFrame);
}

bool CameraView::getFrame(Mat &frame) {
    m_currentFrame.copyTo(frame);
    return true;
}

bool CameraView::esc() {
    if (m_processingThread->esc()) {
        m_esc = true;
    }
    return m_esc;
}