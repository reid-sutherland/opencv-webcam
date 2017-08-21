#include "SharedImageBuffer.h"

using namespace std;

SharedImageBuffer::SharedImageBuffer()
{
    m_nArrived = 0;
    m_doSync = false;
}

void SharedImageBuffer::add(int deviceNumber, Buffer<cv::Mat>* imageBuffer, bool sync)
{
    // Device stream is to be synchronized
    if(sync)
    {
        m_mutex.lock();
        m_syncSet.insert(deviceNumber);
        m_mutex.unlock();
    }
    // Add image buffer to map
    m_imageBufferMap[deviceNumber] = imageBuffer;
}

Buffer<cv::Mat>* SharedImageBuffer::getByDeviceNumber(int deviceNumber)
{
    //if imagebuffer for devicenumber is found
    if (m_imageBufferMap.find(deviceNumber) != m_imageBufferMap.end()) {
        return m_imageBufferMap[deviceNumber];
    }
    else {
        cout << "Error: m_imageBufferMap.find(devicenumber) not found" << endl;
    }
}

void SharedImageBuffer::removeByDeviceNumber(int deviceNumber)
{
    // Remove buffer for device from imageBufferMap
    delete m_imageBufferMap[deviceNumber];
    m_imageBufferMap.erase(deviceNumber);

    // Also remove from syncSet (if present)
    m_mutex.lock();
    if (m_syncSet.count(deviceNumber) > 0)
    {
        m_syncSet.erase(deviceNumber);
        m_cv.notify_all();
    }
    m_mutex.unlock();
}

void SharedImageBuffer::sync(int deviceNumber)
{
    // Only perform sync if enabled for specified device/stream
    unique_lock<mutex> lck(m_mutex);
    if (m_syncSet.count(deviceNumber) > 0) {
        // Increment arrived count
        m_nArrived++;

        // We are the last to arrive: notify all waiting threads
        if (m_doSync && (m_nArrived == m_syncSet.size())) {
            m_cv.notify_all();
        }
        // Still waiting for other streams to arrive: wait
        else {
            m_cv.wait(lck);  //all threads wait until notified
        }

        // Decrement arrived count
        m_nArrived--;
    }
    lck.unlock();
}

void SharedImageBuffer::notifyAll()
{
    m_mutex.lock();
    m_cv.notify_all();
    m_mutex.unlock();
}

void SharedImageBuffer::setSyncEnabled(bool enable)
{
    m_mutex.lock();
    m_doSync = enable;
    m_mutex.unlock();
}

bool SharedImageBuffer::isSyncEnabledForDeviceNumber(int deviceNumber)
{
    return (m_syncSet.count(deviceNumber) > 0);
}

bool SharedImageBuffer::getSyncEnabled()
{
    return m_doSync;
}