#include <vector>
#include <string>
#include <map>

using namespace std;

class CameraProperties {
public:
    CameraProperties();
    ~CameraProperties();

    int getNumberOfCameras() const;
    string getDeviceNameByID(int id);
    vector<int> getDeviceIDs();
    void addDeviceName(string deviceName, int id);
    bool containsDeviceID(int deviceID);

private:
    map<int, string> m_deviceMap;
    vector<int> m_deviceNumbers;
};

CameraProperties::CameraProperties() = default;


CameraProperties::~CameraProperties() = default;


int CameraProperties::getNumberOfCameras() const
{
    return m_deviceMap.size();
}

string CameraProperties::getDeviceNameByID(int id)
{
    return this->m_deviceMap[id];
}

vector<int> CameraProperties::getDeviceIDs()
{
    return this->m_deviceNumbers;
}

void CameraProperties::addDeviceName(string deviceName, int id)
{
    m_deviceMap[id] = deviceName;
    m_deviceNumbers.push_back(id);
}

bool CameraProperties::containsDeviceID(int deviceID) {
    if (m_deviceMap.count(deviceID) > 0) {
        return true;
    }
    return false;
}