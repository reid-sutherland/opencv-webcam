#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "CameraProperties.h"

using namespace std;

class UtilsCameras {
public:
    UtilsCameras();
    ~UtilsCameras();

    CameraProperties* detectCameras();
    void combineImageH(cv::Mat& imgToMerge, cv::Mat frame_left, cv::Mat frame_right);
    string toString(int val);
    string getDeviceInformation(CameraProperties camProperties);
};

UtilsCameras::UtilsCameras(void)
{
}


UtilsCameras::~UtilsCameras(void)
{
}

#ifdef _WIN32
HRESULT UtilsCameras::EnumerateDevices(REFGUID category, IEnumMoniker **ppEnum)
{
    // Create the System Device Enumerator.
    ICreateDevEnum *pDevEnum;
    HRESULT hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pDevEnum));

    if (SUCCEEDED(hr))
    {
        // Create an enumerator for the category.
        hr = pDevEnum->CreateClassEnumerator(category, ppEnum, 0);
        if (hr == S_FALSE)
        {
            hr = VFW_E_NOT_FOUND;  // The category is empty. Treat as an error.
        }
        pDevEnum->Release();
    }
    return hr;
}
#endif

CameraProperties* UtilsCameras::detectCameras(){

    CameraProperties* camerasProp = new CameraProperties();

#ifdef _WIN32
    int i = -1;
    HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);

        if (SUCCEEDED(hr) || hr == RPC_E_CHANGED_MODE) {

            //std::cout << "SUCCESS hr  CoInitializeEx\n";
            IEnumMoniker *pEnum;

            hr = EnumerateDevices(CLSID_VideoInputDeviceCategory, &pEnum);
            //std::cout << "AFTER EnumerateDevices \n---- hr ------"<<hr<<"\n";
            if (SUCCEEDED(hr))
            {
               // std::cout << "SUCCESS EnumerateDevices \n";
                i = DisplayDeviceInformation(pEnum, camerasProp);
                pEnum->Release();
            }

            CoUninitialize();
        }
#endif

#ifdef __linux__

    FILE* pipe = popen("ls /dev/video*", "r");
    if (!pipe) cout << "ERROR Detecting Camera\n";
    char buffer[128];
    string result = "";
    while (!feof(pipe)) {
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;
    }
    pclose(pipe);

    cout << "\nAuto-detecting Video input:\n" << result << "\n";
    int i = 0;
    while (result.find("video") != string::npos)
    {
        std::size_t found = result.find("video");
        int index = atoi(result.substr(found + 5, 1).c_str());
        result = result.substr(found + 5);
        cout << "CAM " << i << " index: " << index << endl;
        camerasProp->addDeviceName(("CAMERA - video" + toString(index)), index);
        i++;
    }

#endif

    return camerasProp;
}

#ifdef _WIN32
int UtilsCameras::DisplayDeviceInformation(IEnumMoniker *pEnum, cameraProperties* camsProp)
{
    IMoniker *pMoniker = NULL;

    while (pEnum->Next(1, &pMoniker, NULL) == S_OK)
    {
        IPropertyBag *pPropBag;
        HRESULT hr = pMoniker->BindToStorage(0, 0, IID_PPV_ARGS(&pPropBag));
        if (FAILED(hr))
        {
            pMoniker->Release();
            continue;
        }

        VARIANT var;
        VariantInit(&var);

        // Get description or friendly name.
        hr = pPropBag->Read(L"Description", &var, 0);
        std::string ch = "Descrption";
        if (FAILED(hr))
        {
            ch = "FriendlyName";
            hr = pPropBag->Read(L"FriendlyName", &var, 0);
        }
        if (SUCCEEDED(hr))
        {
            std::string cameraName = BstrToStdString(var.bstrVal);
            printf("%s: %s ,  Cam N_: %d\n", ch.c_str(), cameraName.c_str(), camsProp->getNumberOfCamera());
            ccConsole::Print("%s: %s ,  Cam N_: %d", ch.c_str(), cameraName.c_str(), camsProp->getNumberOfCamera());
            VariantClear(&var);
            camsProp->addDeviceName(cameraName);
        }

        hr = pPropBag->Write(L"FriendlyName", &var);

        pPropBag->Release();
        pMoniker->Release();
    }
    printf("Number of connected devices %d\n", camsProp->getNumberOfCamera());
    ccConsole::Print("Number of connected devices %d", camsProp->getNumberOfCamera());

    return camsProp->getNumberOfCamera();
}

// convert a BSTR to a std::string.
std::string& UtilsCameras::BstrToStdString(const BSTR bstr, std::string& dst, int cp)
{
    if (!bstr)
    {
        // define NULL functionality. I just clear the target.
        dst.clear();
        return dst;
    }

    // request content length in single-chars through a terminating
    //  nullchar in the BSTR. note: BSTR's support imbedded nullchars,
    //  so this will only convert through the first nullchar.
    int res = WideCharToMultiByte(cp, 0, bstr, -1, NULL, 0, NULL, NULL);
    if (res > 0)
    {
        dst.resize(res);
        WideCharToMultiByte(cp, 0, bstr, -1, &dst[0], res, NULL, NULL);
    }
    else
    {    // no content. clear target
        dst.clear();
    }
    return dst;
}

// conversion with temp.
std::string UtilsCameras::BstrToStdString(BSTR bstr, int cp)
{
    std::string str;
    BstrToStdString(bstr, str, cp);
    return str;
}

#endif

void UtilsCameras::combineImageH(cv::Mat& imgToMerge, cv::Mat frame_left, cv::Mat frame_right)
{
    // Get dimension of final image
    int rows = cv::max(frame_left.rows, frame_right.rows);
    int cols = frame_left.cols + frame_right.cols;
    if (frame_left.channels() == 1)
    {
        cv::cvtColor(frame_left, frame_left, CV_GRAY2RGB);
    }

    if (frame_right.channels() == 1)
    {
        cv::cvtColor(frame_right, frame_right, CV_GRAY2RGB);
    }

    // Create a black image
    cv::Mat3b combine(rows, cols, cv::Vec3b(0,0,0));

    // Copy images in correct position
    frame_left.copyTo(combine(cv::Rect(0, 0, frame_left.cols, frame_left.rows)));
    frame_right.copyTo(combine(cv::Rect(frame_left.cols, 0, frame_right.cols, frame_right.rows)));
    imgToMerge = combine;
}


std::string UtilsCameras::toString(int val){
    std::ostringstream os ;
    os << val ;
    return os.str() ;
}

/*
std::string UtilsCameras::getDeviceInformation(CameraProperties camProperties){
    int i;
    string paragrah = "<br><br>";
    QString qs=QObject::tr("Detected cameras");
    QString dev=QObject::tr("Device Number");
    std::string title = qs.toUtf8().constData();
    std::string devise = dev.toUtf8().constData();
    std::string info = paragrah+"<center><h2><u>"+title+"</u></h2></center>"+paragrah;
    info = info + "<p style='font-size:17px;'>";
    //std::cout << "Info: " << info << "\n";
    for(i=0; i < camProperties.getNumberOfCamera(); i++){
        info = info+"<b> -"+devise+" </b>: " + toString(i) + " - " + camProperties.getDeviceByNumber(i) + paragrah;
        //std::cout << "2-Info: " << info << "\n";
    }
    return info+"</p>";

}
 */