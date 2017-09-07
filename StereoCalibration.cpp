/*
 * StereoCalibration.cpp
 *
 *  Created on: 19 sept. 2015
 *  Author: erman
 */
#include <thread>
#include "StereoCalibration.h"

StereoCalibration StereoCalibration::m_instance = StereoCalibration();

StereoCalibration::StereoCalibration()
{

}

StereoCalibration::~StereoCalibration()
{

}

StereoCalibration& StereoCalibration::Instance()
{
    if(!m_instance.isInstantiated())
    {
        m_instance.init();
    }
    return m_instance;
}


// This function saves the calibration parameters of the cameras to a file.
bool StereoCalibration::saveCalib()
{
    cv::FileStorage fs(this->calib_filename, cv::FileStorage::WRITE);

    if (!fs.isOpened())
    {
        std::cout << "Camera Calibration file " << this->calib_filename << " is not found." << std::endl;
        return false;
    }

    fs << "CM1" << this->CM1;
    fs << "CM2" << this->CM2;
    fs << "D1" << this->D1;
    fs << "D2" << this->D2;

    fs << "R" << this->R;
    fs << "T" << this->T;
    fs << "E" << this->E;
    fs << "F" << this->F;

    fs << "R1" << this->R1;
    fs << "R2" << this->R2;
    fs << "P1" << this->P1;
    fs << "P2" << this->P2;
    fs << "Q" << this->Q;

    fs << "framesize" << this->framesize;
    fs << "calib_error" << this->calib_error;
    fs << "validRoiL" << this->validRoi[0];
    fs << "validRoiR" << this->validRoi[1];

    fs.release();
    std::cout << "Calibration matrices saved to " << this->calib_filename.c_str() << std::endl << std::endl;

    return true;
}


// This function loads the calibration paremeters of the cameras from a file.
bool StereoCalibration::loadCalib()
{
    cv::FileStorage fs(this->calib_filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        std::cout << "Camera Calibration file " << this->calib_filename.c_str() << " is not found." << std::endl;
        return false;
    }

    fs["CM1"] >> this->CM1;
    fs["CM2"] >> this->CM2;
    fs["D1"] >> this->D1;
    fs["D2"] >> this->D2;

    fs["R"] >> this->R;
    fs["T"] >> this->T;
    fs["E"] >> this->E;
    fs["F"] >> this->F;

    fs["R1"] >> this->R1;
    fs["R2"] >> this->R2;
    fs["P1"] >> this->P1;
    fs["P2"] >> this->P2;

    fs["Q"] >> this->Q;

    fs["framesize"] >> this->framesize;
    fs["calib_error"] >> this->calib_error;
    fs["validRoiL"] >> this->validRoi[0];
    fs["validRoiR"] >> this->validRoi[1];

    fs.release();
    std::cout << "Calibration matrices successfully loaded from " << this->calib_filename.c_str() << std::endl << std::endl;
    this->calib_param_loaded = true;

    return true;
}


bool StereoCalibration::readStringList( const std::string& filename, std::vector<std::string>& fileNameList )
{
    printf("Reading file from: %s \n", filename.c_str());
    fileNameList.resize(0);
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if( !fs.isOpened() )
        return false;
    cv::FileNode n = fs.getFirstTopLevelNode();
    if( n.type() != cv::FileNode::SEQ )
        return false;
    cv::FileNodeIterator it = n.begin(), it_end = n.end();
    for( ; it != it_end; ++it )
        fileNameList.push_back((std::string)*it);
    return true;
}


// This function allows you to read from a file the parameters of the calibration in a file
int StereoCalibration::stereoChessDetection(cv::Mat& frame_left, cv::Mat& frame_right, cv::Mat& resframe_left, cv::Mat& resframe_right)
{
    cv::Size board_sz = cv::Size(this->board_w, this->board_h);  //Checkerboard Dimensions
    //int board_n = this->board_w*this->board_h;  //Num Corners
    int result = 0;

    //Ensure frames contain data to avoid errors.
    if (!frame_left.data || !frame_right.data)
    {
        std::cerr << "Error: in SC::stereoChessDetection - no data in frame" << std::endl;
        return -1;
    }

    if( frame_left.size() == frame_right.size() )
    {
        if( framesize == cv::Size() || frame_left.size() == framesize ){
            framesize = frame_left.size();
            std::cout << "Framesize: " << frame_left.size() << std::endl;
          }
        else
        {
            std::cerr << "Warning: The images has the size different from the first image size. Skipping the pair." << std::endl;
            return -1;
        }
    }
    else
    {
        std::cerr << "Warning: The left and right images have different sizes." << std::endl;
        return -1;
    }

    bool found_left = false; //value noting if checkerboard found in image
    bool found_right = false; //value noting if checkerboard found in image

    //Vectors for holding point information used in camera calibration
    std::vector<cv::Point3f> object;
    std::vector<cv::Point2f> corners_left;
    std::vector<cv::Point2f> corners_right;

    //Mat objects for holding camera images
    cv::Mat gray_left;
    cv::Mat gray_right;
    frame_left.copyTo(resframe_left);
    frame_right.copyTo(resframe_right);

    const float squareSize = 0.023f;  // Set this to your actual square size
    for(int j = 0; j < board_h; j++ )
        for(int k = 0; k < board_w; k++ )
            object.push_back(cv::Point3f(k*squareSize, j*squareSize, 0));

    if(frame_left.channels()>1)
        cv::cvtColor(frame_left, gray_left, CV_BGR2GRAY);
    else gray_left = frame_left;
    if(frame_right.channels()>1)
        cv::cvtColor(frame_right, gray_right, CV_BGR2GRAY);
    else gray_right = frame_right;

    //Use converted images to search for checkerboard. Returns true if found.
    found_left = cv::findChessboardCorners(gray_left, board_sz, corners_left,
                                           cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
                                           //CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    found_right = cv::findChessboardCorners(gray_right, board_sz, corners_right,
                                            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);

    //checkerboard found in both images
    if (found_left && found_right)//Sub pixel optimization for corner locations.
    {
        //! adjusts the corner locations with sub-pixel accuracy to maximize the certain cornerness criteria
        cv::cornerSubPix(gray_left, corners_left, cv::Size(11,11), cv::Size(-1,-1),
                         cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01));
        cv::cornerSubPix(gray_right, corners_right, cv::Size(11,11), cv::Size(-1,-1),
                         cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01));

        //! draw checkerboard corners and lines on RGB images.
        cv::drawChessboardCorners(resframe_left, board_sz, corners_left, found_left);
        cv::drawChessboardCorners(resframe_right, board_sz, corners_right, found_right);

        this->imagePoints_left.push_back(corners_left);
        this->imagePoints_right.push_back(corners_right);
        this->objectPoints.push_back(object);
        nimages++;

        result = 0;
    } else
        return -1;

    std::cout << "Number of calibration frames captured: " << nimages << std::endl;
    std::cout << "Capture successful." << std::endl << std::endl;
    return result;
}

// This function allows to calibrate the cameras and to save the calibration parameters in a file
// (Intrinsic and extrinsic parameters)
int StereoCalibration::stereoCalib(bool saveResult)
{
    std::cout << "\n***Running stereo calibration...***\n" << std::endl;

    if( nimages < min_poses )
    {
        std::cout << "Error: too little pairs to run the calibration." << std::endl;
        return -1;
    }

    CM1 = cv::initCameraMatrix2D(objectPoints,imagePoints_left,framesize,0);
    CM2 = cv::initCameraMatrix2D(objectPoints,imagePoints_right,framesize,0);

    double rms = cv::stereoCalibrate(objectPoints, imagePoints_left,imagePoints_right,
                                     CM1, D1, CM2, D2,
                                     framesize, R, T, E, F,
                                     cv::CALIB_FIX_ASPECT_RATIO +
                                     cv::CALIB_ZERO_TANGENT_DIST +
                                     cv::CALIB_USE_INTRINSIC_GUESS +
                                     cv::CALIB_SAME_FOCAL_LENGTH +
                                     cv::CALIB_RATIONAL_MODEL +
                                     cv::CALIB_FIX_K3 + cv::CALIB_FIX_K4 + cv::CALIB_FIX_K5,
                                     cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 1e-5) );

    calib_error = rms;
    printf("Done with RMS error %.2lf \n", rms);

    // CALIBRATION QUALITY CHECK
    // because the output fundamental matrix implicitly
    // includes all the output information,
    // we can check the quality of calibration using the
    // epipolar geometry constraint: m2^t*F*m1=0
    double err = 0;
    int npoints = 0;
    std::vector<cv::Vec3f> lines[2];
    for(int i = 0; i < nimages; i++ )
    {
        int npt = (int)imagePoints_left[i].size();
        cv::Mat imgpt[2];

        imgpt[0] = cv::Mat(imagePoints_left[i]);
        imgpt[1] = cv::Mat(imagePoints_right[i]);
        cv::undistortPoints(imgpt[0], imgpt[0], CM1, D1, cv::Mat(), CM1);
        cv::undistortPoints(imgpt[1], imgpt[1], CM2, D2, cv::Mat(), CM2);
        cv::computeCorrespondEpilines(imgpt[0], 1, F, lines[0]);
        cv::computeCorrespondEpilines(imgpt[1], 2, F, lines[1]);

        for(int j = 0; j < npt; j++ )
        {
            double errij = fabs(imagePoints_left[i][j].x*lines[1][j][0] +
                    imagePoints_left[i][j].y*lines[1][j][1] + lines[1][j][2]) +
                    fabs(imagePoints_right[i][j].x*lines[0][j][0] +
                    imagePoints_right[i][j].y*lines[0][j][1] + lines[0][j][2]);
            err += errij;
        }
        npoints += npt;
    }
    printf("Average epipolar err = %.2lf \n", err/npoints);

    cv::stereoRectify(CM1, D1, CM2, D2, framesize, R, T, R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, 0, framesize, &validRoi[0], &validRoi[1]);


    // COMPUTE AND DISPLAY RECTIFICATION

    // IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
        // we already computed everything
    }
    // OR ELSE HARTLEY'S METHOD
    else
    {
        // use intrinsic parameters of each camera, but
        // compute the rectification transformation directly
        // from the fundamental matrix
        std::vector<cv::Point2f> allimgpt[2];

        for(int i = 0; i < nimages; i++ )
        {
            std::copy(imagePoints_left[i].begin(), imagePoints_left[i].end(), back_inserter(allimgpt[0]));
            std::copy(imagePoints_right[i].begin(), imagePoints_right[i].end(), back_inserter(allimgpt[1]));
        }

        F = cv::findFundamentalMat(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), cv::FM_8POINT, 0, 0);
        cv::Mat H1, H2;
        cv::stereoRectifyUncalibrated(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), F, framesize, H1, H2, 3);

        R1 = CM1.inv()*H1*CM1;
        R2 = CM2.inv()*H2*CM2;
        P1 = CM1;
        P2 = CM2;
    }
    printf("\n***Calibration complete. RMSE = %.2lf***\n\n", rms);

    // Tell that the calibration parameters is available
    calib_param_loaded = true;
    if (saveResult){
        saveCalib();
    }

    return 0;
}

// This function rectifies the input images and returns the rectified images (by reference)
int StereoCalibration::rectifyStereoImg(cv::Mat imgLeft, cv::Mat imgRight,
                                        cv::Mat& rectImgLeft, cv::Mat& rectImgRight)
{
    cv::Mat map1x, map1y, map2x, map2y;

    //Precompute maps for cv::remap()
    cv::initUndistortRectifyMap(CM1, D1, R1, P1, framesize, CV_16SC2, map1x, map1y);
    cv::initUndistortRectifyMap(CM2, D2, R2, P2, framesize, CV_16SC2, map2x, map2y);

    if(map1x.size() == cv::Size() || map1y.size() == cv::Size()
            || map2x.size() == cv::Size() || map2y.size() == cv::Size())
    {
        std::cerr << "Error: Rectification of image failed." << std::endl;
        return -1;
    }

    //! Applaying rectification of images
    cv::remap(imgLeft, rectImgLeft, map1x, map1y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::remap(imgRight, rectImgRight, map2x, map2y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

    return 0;
}

// This function is to change the calibration path name
void StereoCalibration::setCalibFilename(std::string _calib_filename)
{
    this->calib_filename = _calib_filename;
}

void StereoCalibration::printQMatrix() {
    std::cout << "Cx = " << -1*Q.at<double>(0, 3) << std::endl;
    std::cout << "Cy = " << -1*Q.at<double>(1, 3) << std::endl;
    std::cout << "f = " << Q.at<double>(2, 3) << std::endl;
    double Tx = -1/Q.at<double>(3, 2);
    std::cout << "Tx = " << Tx << std::endl;
    std::cout << "(Cx - C'x) = " << Tx*Q.at<double>(3, 3) << std::endl << std::endl;

/*

Q Matrix form:

    | 1  0    0         -cx      |
Q = | 0  1    0         -cy      |
    | 0  0    0          f       |
    | 0  0  -1/Tx  (cx - c'x)/Tx |

    cx / cy     coordinates of the principal point in the dominant camera
    c'x         x-coordinate of the principal point in the non-dominant camera
                    (will be equal to cx if CALIB_ZERO_DISPARITY is specified, which is normally the case here)
    f           focal length
    Tx          baseline length - translation from one optical center to the other

    Orientation: cx is at (0,3), cy is at (1,3), etc.
*/
}