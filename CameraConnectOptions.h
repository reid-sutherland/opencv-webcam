//
// Created by reid on 8/8/17.
//

#ifndef CAMERACONNECTOPTIONS_H
#define CAMERACONNECTOPTIONS_H

using namespace std;

class CameraConnectOptions {
public:
    CameraConnectOptions(int deviceNumber, int bufferSize, bool enableFrameDrop) :
        m_deviceNumber(deviceNumber),
        m_bufferSize(bufferSize),
        m_enableFrameDrop(enableFrameDrop)
    { }
    ~CameraConnectOptions() { }

    int getDeviceNumber()
    {
        return m_deviceNumber;
    }
    int getBufferSize()
    {
        return m_bufferSize;
    }
    bool getEnableFrameDrop()
    {
        return m_enableFrameDrop;
    }

private:
    int m_deviceNumber;
    int m_bufferSize;
    bool m_enableFrameDrop;
};


#endif //CAMERACONNECTOPTIONS_H
