#ifndef SHAREDIMAGEBUFFER_H
#define SHAREDIMAGEBUFFER_H

#include <set>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <map>
#include <opencv2/core/mat.hpp>

#include "Buffer.h"

using namespace std;

class SharedImageBuffer
{
public:
    SharedImageBuffer();
    void add(int deviceNumber, Buffer<cv::Mat> *imageBuffer, bool sync = false);
    Buffer<cv::Mat>* getByDeviceNumber(int deviceNumber);
    void removeByDeviceNumber(int deviceNumber);
    void sync(int deviceNumber);
    void notifyAll();
    void setSyncEnabled(bool enable);
    bool isSyncEnabledForDeviceNumber(int deviceNumber);
    bool getSyncEnabled();

private:
    //unordered_map<int, Buffer<cv::Mat>*> m_imageBufferMap;
    map<int, Buffer<cv::Mat>*> m_imageBufferMap;
    set<int> m_syncSet;
    condition_variable m_cv;
    mutex m_mutex;
    int m_nArrived;
    bool m_doSync;
};

#endif // SHAREDIMAGEBUFFER_H