#ifndef __TYCAMCAPTURE_H
#define __TYCAMCAPTURE_H

#include "common/common.hpp"
#include "TYImageProc.h"

class tycamCapture{
    public:
        tycamCapture();
        ~tycamCapture(){
            ASSERT_OK( TYStopCapture(hDevice) );
            ASSERT_OK( TYCloseDevice(hDevice) );
            ASSERT_OK( TYCloseInterface(hIface) );
            ASSERT_OK( TYDeinitLib() );
            delete frameBuffer[0];
            delete frameBuffer[1];
        }

        void doRegister(const TY_CAMERA_CALIB_INFO& depth_calib
                      , const TY_CAMERA_CALIB_INFO& color_calib
                      , const cv::Mat& depth
                      , const cv::Mat& color
                      , bool needUndistort
                      , cv::Mat& undistort_color
                      , cv::Mat& out
                      , bool map_depth_to_color);
        void captureFrame();

    private:
        TY_INTERFACE_HANDLE hIface;
        TY_DEV_HANDLE hDevice;
        std::string ID, IP;
        bool hasUndistortSwitch;
        bool hasDistortionCoef;
        uint32_t frameSize;
        char* frameBuffer[2];
        TY_TRIGGER_PARAM trigger;
        TY_FRAME_DATA frame;
        TY_ISP_HANDLE isp_handle;

        TY_CAMERA_CALIB_INFO depth_calib;
        TY_CAMERA_CALIB_INFO color_calib;
};

#endif