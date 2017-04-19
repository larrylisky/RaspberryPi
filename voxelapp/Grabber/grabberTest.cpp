/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#include <CameraSystem.h>
#include <Common.h>
#include <thread>
#include "Grabber.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace Voxel;
using namespace cv;

#define ESC             27
#define PROFILE_NAME    "Calibrated High Power"

void frameUpdate(Grabber *g)
{
    int key;
    int rows, cols;
    XYZIPointCloudFrame frame;
  
            
    g->getFrameSize(rows, cols);
    
    Mat zMat = Mat::zeros(rows, cols, CV_32FC1);
    Mat iMat = Mat::zeros(rows, cols, CV_32FC1);
    
    namedWindow("Amplitude", WINDOW_NORMAL);
    namedWindow("Z-Depth", WINDOW_NORMAL);
       
    if (g->getXYZIFrame(&frame)) 
    {            
        for (auto i = 0; i < rows; i++) 
        {
            for (auto j = 0; j < cols; j++) 
            {
                int idx = i * cols +j;
                zMat.at<float>(i,j) = frame.points[idx].z;
                iMat.at<float>(i,j) = frame.points[idx].i;
            }
        }
        
        imshow("Amplitude", iMat);
        imshow("Z-Depth", zMat);
        
        key = waitKey(33);
    }
            
    if (key == 'q') 
    {
        g->updateExit();
    }
}


int main(int argc, char **argv)
{
    logger.setDefaultLogLevel(LOG_INFO);

    CameraSystem sys;
    DepthCameraPtr dc; 
      
    Voxel::String profileName = Voxel::String(PROFILE_NAME);
  
    if (argc == 2) 
    {
        profileName = Voxel::String(argv[1]);
    }
    cout << "Profile=" << profileName << endl;

    vector<DevicePtr> devices = sys.scan();

    if (devices.size() > 0)
        dc = sys.connect(devices[0]);

    if (!dc) 
    {
        logger(LOG_ERROR) << "grabberTest: no camera connected." << endl;
        return -1;
    }
    
    Grabber *_grabber = 
        new Grabber(dc, DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME, sys);
        
    if (!_grabber->setProfile(profileName)) 
    {
        const Map<int, Voxel::String> &profiles = _grabber->getProfiles();
            
        logger(LOG_ERROR) << "Profile: " << PROFILE_NAME 
                            << " not found; choose from:" << endl;
                            
        cout << "Choose from below: " << endl;
        
        for (auto &p: profiles) 
        {
            cout << " => " << p.second << endl;
        }
        return -1;
    }
    
    _grabber->setFrameRate(30.0);
    _grabber->registerUpdate(frameUpdate);
    _grabber->run();
}


