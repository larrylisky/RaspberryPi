/*
 * TI Voxel Lib component.
 *
 * Copyright (c) 2014 Texas Instruments Inc.
 */

#ifndef GRABBER_H
#define GRABBER_H

#include <deque>
#include <string>
#include <CameraSystem.h>
#include <DepthCamera.h>
#include <Common.h>
#include <unistd.h>
#include <termio.h>
#include <functional>
#include <stdint.h>

using namespace std;
using namespace Voxel;

class Grabber
{
protected:
    DepthCameraPtr _depthCamera;
    
    CameraSystem &_sys;

    Mutex _mtx;
    
    deque<Frame *> _qFrame;
    
    DepthCamera::FrameType _frameType;

    uint32_t _frameCount;  
    
    bool _updateDone;
    
    int _rows, _cols;
    
    std::function<void(Grabber *g)> _update;
    
    void _callback(DepthCamera &depthCamera, const Frame &frame, 
            DepthCamera::FrameType type);

    void _applyFilter();

public:
    Grabber(DepthCameraPtr depthCamera, DepthCamera::FrameType frameType, 
            CameraSystem &sys);

    virtual float getFramesPerSecond() const
    {
        FrameRate r;
        if(!_depthCamera->getFrameRate(r))
      	    return 0;
    	else
      	    return r.getFrameRate();
  	}

    virtual std::string getName() const { return _depthCamera->id(); }

    virtual bool isRunning() const { return _depthCamera->isRunning(); }

    virtual void start() { _depthCamera->start(); }

    virtual void stop() { _depthCamera->stop(); _depthCamera->wait(); }

    virtual uint32_t getFrameCount();
    
    virtual bool getXYZIFrame(XYZIPointCloudFrame *f);
    
    virtual bool getDepthFrame(DepthFrame *f);
    
    virtual void setFrameRate(float frameRate);
    
    virtual void getFrameSize(int &rows, int &cols) { rows=_rows; cols=_cols; }
    
    virtual const Map<int, Voxel::String> &getProfiles();

    virtual bool setProfile(Voxel::String name);
    
    virtual bool setRegister(Voxel::String name, uint value)
                    { return _depthCamera->set(name, value); }
                    
    virtual bool isEmpty();    
    
    virtual void registerUpdate(std::function<void(Grabber *g)> update);
    
    virtual void updateExit() { stop(); _updateDone = true; }
    
    virtual void run();
    
    ~Grabber()
    {
        if(isRunning())
            stop();
    }

};


#endif // GRABBER_H
