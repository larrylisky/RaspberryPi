/*
 * TI Grabber component.
 *
 * Copyright (c) 2017 Texas Instruments Inc.
 */

#include "Grabber.h"

#define FIFO_SIZE       3

Grabber::Grabber(DepthCameraPtr depthCamera, DepthCamera::FrameType frameType,
                CameraSystem &sys) : 
        _depthCamera(depthCamera), _frameType(frameType), _sys(sys)
{  
    if (!_depthCamera->isInitialized())
    {
        logger(LOG_ERROR) << "Grabber: camera not initialized." << endl;
        return;
    }

    FrameSize sz;
    _depthCamera->getFrameSize(sz);
    
    _rows = sz.height;
    _cols = sz.width;
    
    _frameCount = 0;
    
    _updateDone = false;
    
    _applyFilter();

    _depthCamera->registerCallback(_frameType, std::bind(&Grabber::_callback,
                    this, std::placeholders::_1, std::placeholders::_2, 
                    std::placeholders::_3));
}



uint32_t Grabber::getFrameCount()
{ 
    Lock<Mutex> _(_mtx);
    
    return _frameCount;
}


bool Grabber::getDepthFrame(DepthFrame *frame)
{
    bool rc = false;
        
    Lock<Mutex> _(_mtx);
 
    if (_qFrame.size() > 0 &&
        _frameType == DepthCamera::FRAME_DEPTH_FRAME)
    {
        Frame *f = _qFrame.front();
	    *frame = *(dynamic_cast<DepthFrame *>(f));
	    rc = true;
	}
	
    return rc;
}


bool Grabber::getXYZIFrame(XYZIPointCloudFrame *frame)
{   
    bool rc = false;
    
    Lock<Mutex> _(_mtx);
 
    if (_qFrame.size() > 0 && 
        _frameType == DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME)
    {
        Frame *f = _qFrame.front();
	    *frame = *(dynamic_cast<XYZIPointCloudFrame *>(f));
	    rc = true;
	}
	
    return rc;
}


void Grabber::setFrameRate(float frameRate)
{
    FrameRate r;
    
    r.denominator = 10000;
    r.numerator = int(frameRate)*10000;
    int g = gcd(r.numerator, r.denominator);
    r.numerator /= g;
    r.denominator /= g;
    
    _depthCamera->setFrameRate(r);
}


bool Grabber::isEmpty()
{
    Lock<Mutex> _(_mtx);
    
    return (_qFrame.empty());
}


void Grabber::registerUpdate(std::function<void(Grabber *g)> update)
{
    _update = update;
}


void Grabber::run()
{
    start();
    
    _updateDone = false;
    
    while (!_updateDone)
    {
        _update(this);
    }
}


const Map<int, Voxel::String> &Grabber::getProfiles()
{
    return _depthCamera->getCameraProfileNames();
}


bool Grabber::setProfile(Voxel::String name)
{
    bool rc = false;
    const Map<int, Voxel::String> &profiles = 
                            _depthCamera->getCameraProfileNames();
    for (auto &p: profiles) 
    {
        if (p.second == name) 
        {
            int profile_id = p.first;
            
            ConfigurationFile *c = 
                _depthCamera->configFile.getCameraProfile(p.first);
                
            if (c && c->getLocation() == ConfigurationFile::IN_CAMERA) 
            {
                if (_depthCamera->setCameraProfile(profile_id)) 
                {
                    rc = true;
                    break;
                }
            }
        }
    }
    return rc;
}


//=======  Protected methods  ========

void Grabber::_applyFilter()
{
    FilterPtr p = _sys.createFilter("Voxel::MedianFilter", 
                            DepthCamera::FRAME_RAW_FRAME_PROCESSED);
    if (!p)
    {
        logger(LOG_ERROR) << "Failed to get MedianFilter" << std::endl;
        return;
    }
    p->set("deadband", 0.0f);
    _depthCamera->addFilter(p, DepthCamera::FRAME_RAW_FRAME_PROCESSED);
}


void Grabber::_callback(DepthCamera &depthCamera, const Frame &frame, 
				DepthCamera::FrameType type)
{
    Frame *nf;

    Lock<Mutex> _(_mtx);
    
    if (type == DepthCamera::FRAME_DEPTH_FRAME)
    {
        if (_qFrame.size() >= FIFO_SIZE) 
        {
            nf = _qFrame.front();
            _qFrame.pop_front();
            delete dynamic_cast<const DepthFrame *>(nf);
        }
        
        const DepthFrame *f = dynamic_cast<const DepthFrame *>(&frame);
        nf = dynamic_cast<Frame *>(new DepthFrame(*f));
        _qFrame.push_back(nf);
        
        _frameCount++;
    }
    else if(type == DepthCamera::FRAME_XYZI_POINT_CLOUD_FRAME)
    {
    	if (_qFrame.size() >= FIFO_SIZE) 
    	{
    	    nf = _qFrame.front();
    	    _qFrame.pop_front();
            delete dynamic_cast<const XYZIPointCloudFrame *>(nf);
        }
            
        const XYZIPointCloudFrame *f 
	            = dynamic_cast<const XYZIPointCloudFrame *>(&frame);
        nf = dynamic_cast<Frame *>(new XYZIPointCloudFrame(*f));
        _qFrame.push_back(nf);
        
        _frameCount++;
    }
    else 
    {
        logger(LOG_ERROR) << "Grabber: unknown callback type: " << type << endl;
    }
}



