#include "tycamCapture.h"

void eventCallback(TY_EVENT_INFO *event_info, void *userdata)
{
    if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE) {
        LOGD("=== Event Callback: Device Offline!");
        // Note: 
        //     Please set TY_BOOL_KEEP_ALIVE_ONOFF feature to false if you need to debug with breakpoint!
    }
    else if (event_info->eventId == TY_EVENT_LICENSE_ERROR) {
        LOGD("=== Event Callback: License Error!");
    }
}

tycamCapture::tycamCapture()
{
    hIface = NULL;
    hDevice = NULL;

    LOGD("=== Init lib");
    ASSERT_OK( TYInitLib() );
    TY_VERSION_INFO ver;
    ASSERT_OK( TYLibVersion(&ver) );
    LOGD("     - lib version: %d.%d.%d", ver.major, ver.minor, ver.patch);

    std::vector<TY_DEVICE_BASE_INFO> selected;
    ASSERT_OK( selectDevice(TY_INTERFACE_ALL, ID, IP, 1, selected) );
    ASSERT_OK( selected.size() > 0 );

    TY_DEVICE_BASE_INFO& selectedDev = selected[0];

    ASSERT_OK( TYOpenInterface(selectedDev.iface.id, &hIface) );
    ASSERT_OK( TYOpenDevice(hIface, selectedDev.id, &hDevice) );

    int32_t allComps;
    ASSERT_OK( TYGetComponentIDs(hDevice, &allComps) );
    if(!(allComps & TY_COMPONENT_RGB_CAM)){
        LOGE("=== Has no RGB camera, cant do registartion");
    }

    ASSERT_OK( TYISPCreate(&isp_handle) );
    ASSERT_OK( ColorIspInitSetting(isp_handle, hDevice));

    LOGD("=== Configure components");
    int32_t componentIDs = TY_COMPONENT_DEPTH_CAM | TY_COMPONENT_RGB_CAM;
    ASSERT_OK( TYEnableComponents(hDevice, componentIDs) );

    ASSERT_OK( TYHasFeature(hDevice, TY_COMPONENT_RGB_CAM, TY_BOOL_UNDISTORTION, &hasUndistortSwitch) );
    ASSERT_OK( TYHasFeature(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_DISTORTION, &hasDistortionCoef) );
    if (hasUndistortSwitch)
    {
        ASSERT_OK( TYSetBool(hDevice, TY_COMPONENT_RGB_CAM, TY_BOOL_UNDISTORTION, true) );
    }

    LOGD("=== Prepare image buffer");
    ASSERT_OK( TYGetFrameBufferSize(hDevice, &frameSize) );
    LOGD("     - Get size of framebuffer, %d", frameSize);
    LOGD("     - Allocate & enqueue buffers");
    frameBuffer[0] = new char[frameSize];
    frameBuffer[1] = new char[frameSize];
    LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[0], frameSize);
    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[0], frameSize) );
    LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[1], frameSize);
    ASSERT_OK( TYEnqueueBuffer(hDevice, frameBuffer[1], frameSize) );

    LOGD("=== Register event callback");
    ASSERT_OK( TYRegisterEventCallback(hDevice, eventCallback, NULL) );

    LOGD("=== Set trigger to slave mode");
    trigger.mode = TY_TRIGGER_MODE_SLAVE;
    ASSERT_OK( TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &trigger, sizeof(trigger)) );

    LOGD("=== Read depth calib info");
    ASSERT_OK( TYGetStruct(hDevice, TY_COMPONENT_DEPTH_CAM, TY_STRUCT_CAM_CALIB_DATA
          , &depth_calib, sizeof(depth_calib)) );

    LOGD("=== Read color calib info");
    ASSERT_OK( TYGetStruct(hDevice, TY_COMPONENT_RGB_CAM, TY_STRUCT_CAM_CALIB_DATA
          , &color_calib, sizeof(color_calib)) );

    LOGD("Start Capture");
    ASSERT_OK( TYStartCapture(hDevice) );
}

void tycamCapture::captureFrame()
{
    int err = TYFetchFrame(hDevice, &frame, 2000);
    if( err != TY_STATUS_OK )
    {
        LOGE("Fetch frame error %d: %s", err, TYErrorString(err));
        return;
    }

    cv::Mat depth, color;
    parseFrame(frame, &depth, 0, 0, &color, isp_handle);
    if(depth.empty())
    {
        LOGE("Depth Map Empty");
    }

    if(color.empty())
    {
        LOGE("Color Map Empty");
    }

    if (!depth.empty() && !color.empty())
    {
        cv::Mat undistort_color, out;
        bool needUndistort = !hasUndistortSwitch && hasDistortionCoef;
        if(needUndistort)
        {
            doRegister(depth_calib, color_calib, depth, color, needUndistort, undistort_color, out, 1);
        }
        else
        {
            undistort_color = color;
            out = color;
            LOGD("Do not undistort Image");
        }
    }
}

void tycamCapture::doRegister(const TY_CAMERA_CALIB_INFO& depth_calib
                            , const TY_CAMERA_CALIB_INFO& color_calib
                            , const cv::Mat& depth
                            , const cv::Mat& color
                            , bool needUndistort
                            , cv::Mat& undistort_color
                            , cv::Mat& out
                            , bool map_depth_to_color
                            )
{
  // do undistortion
  if (needUndistort) {
    TY_IMAGE_DATA src;
    src.width = color.cols;
    src.height = color.rows;
    src.size = color.size().area() * 3;
    src.pixelFormat = TY_PIXEL_FORMAT_RGB;
    src.buffer = color.data;

    undistort_color = cv::Mat(color.size(), CV_8UC3);
    TY_IMAGE_DATA dst;
    dst.width = color.cols;
    dst.height = color.rows;
    dst.size = undistort_color.size().area() * 3;
    dst.buffer = undistort_color.data;
    dst.pixelFormat = TY_PIXEL_FORMAT_RGB;
    ASSERT_OK(TYUndistortImage(&color_calib, &src, NULL, &dst));
  }
  else {
    undistort_color = color;
  }

  // do register
  if (map_depth_to_color) {
    out = cv::Mat::zeros(undistort_color.size(), CV_16U);
    ASSERT_OK(
      TYMapDepthImageToColorCoordinate(
        &depth_calib,
        depth.cols, depth.rows, depth.ptr<uint16_t>(),
        &color_calib,
        out.cols, out.rows, out.ptr<uint16_t>()
      )
    );
    cv::Mat temp;
    //you may want to use median filter to fill holes in projected depth image
    //or do something else here
    cv::medianBlur(out, temp, 5);
    out = temp;
  }
  else {
    out = cv::Mat::zeros(depth.size(), CV_8UC3);
    ASSERT_OK(
      TYMapRGBImageToDepthCoordinate(
        &depth_calib,
        depth.cols, depth.rows, depth.ptr<uint16_t>(),
        &color_calib,
        undistort_color.cols, undistort_color.rows, undistort_color.ptr<uint8_t>(),
        out.ptr<uint8_t>()
      )
    );
  }
}