// #ifndef MINDCAM_HPP
// #define MINDCAM_HPP

// #include <CameraApi.h>

// #include <camera_info_manager/camera_info_manager.hpp>
// #include <image_transport/image_transport.hpp>
// #include <rclcpp/logging.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/utilities.hpp>
// #include <sensor_msgs/msg/camera_info.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <mutex>

// #include <opencv2/opencv.hpp>
// #include <opencv2/imgproc.hpp>


// namespace mind_camera {

// struct MindParams {
//     std::atomic<double> bullet_velocity;
//     std::atomic<bool> right_press;
//     std::atomic<bool> lobshot;
// };

// class MindCameraNode : public rclcpp::Node {
// public:
//     explicit MindCameraNode(const rclcpp::NodeOptions& options);
//     ~MindCameraNode() override;

//     bool setCameraParam();

// private:
//   // 曝光时间
//     double mExposureTime;
//     // R gain
//     int mRGainInt;
//     // G gain0
//     int mGGainInt;
//     // B Gain
//     int mBGainInt;
//     // 相机伽马
//     int mCameraGamma;
//     // // 相机的句柄
//     // int mCameraHandle;
//     // 相机的编号 不是句柄
//     int mCameraNumber;
//     int mFrameSpeedInt;
//     int mCameraCountsInt = 10;
//     int mChannel = 3;
//     // 曝光增益
//     double mValue;
//     // Mat mSrc;
//     // Mat mCameraMatrix=Mat(3,3,CV_64FC1);
//     // Mat mDistCoeffs=Mat(5,1,CV_64FC1);
//     BYTE* mPbyBuffer;
//     // 处理后数据缓存区
//     unsigned char* mGpRgbBuffer;
//     // IplImage* mIplImage;

//     tSdkImageResolution mResolution;
//     tSdkCameraDevInfo mCameraEnumList[10];
//     // tSdkFrameHead mFrameHead;
//     // 设备描述信息
//     tSdkCameraCapbility mCapability;
//     std::string frame_id;
//     int nRet = CAMERA_STATUS_SUCCESS;

//     // void* camera_handle;
//     int mCameraHandle;

//     std::thread capture_thread;
//     std::thread monitor_thread;
//     MindParams params;
//     //相机图像发布
//     image_transport::CameraPublisher image_pub;

//      tSdkFrameHead img_info;//图像信息
//     // tSdkImageConvertParam convert_param;//像素格式
    
//     // 相机发布
//     std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager;
//     sensor_msgs::msg::CameraInfo camera_info_msg;

//     //处理参数客户端和订阅部分
//     rclcpp::AsyncParametersClient::SharedPtr param_client;
//     rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub;
   
//     //错误计数器和相机状态标志
//     int fail_cnt = 0;
//     bool rotate_180 = false;
//     std::atomic<bool> grab_on = false;
//     std::atomic<bool> monitor_on = false;
//     std::atomic<bool> camera_failed = false;
//     std::atomic<bool> param_changed = false;

//     // 只在模式切换的时候更改，避免频繁读写
//     void declare_params();
//     void initParamsBySDK();
//     void reset();
//     void open_device();
//     void close_device();
//     void start_grab();
//     void stop_grab();
//     void set_md_params();
//     void set_grab_params(int offset_x, int offset_y, int roi_width, int roi_height);
//     void grab();
//     void monitor();
//     std::pair<int, int> get_sensor_height_width();
// };

// class CameraException : public std::exception {
//    public:
//     std::string info;
//     CameraException(const std::string&& _info) : info{_info} {}
//     const char* what() const noexcept { return info.c_str(); }
// };

// }  

// #endif

#ifndef MINDCAM_HPP
#define MINDCAM_HPP
// MindVision SDK
#include <CameraApi.h>
// ROS
// #include <rm_utils/data.h>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
// #include <rm_interfaces/msg/rmrobot.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <mutex>

namespace mind_camera {

struct MindParams {
    std::atomic<double> bullet_velocity;
    std::atomic<bool> right_press;
    std::atomic<bool> lobshot;
};

class MindCameraNode : public rclcpp::Node {
public:
    explicit MindCameraNode(const rclcpp::NodeOptions& options);
    ~MindCameraNode() override;

private:
    std::string frame_id;
    int nRet = CAMERA_STATUS_SUCCESS;
    CameraHandle camera_handle;
    std::thread capture_thread;
    std::thread monitor_thread;
    MindParams params;
    // 相机图像发布
    image_transport::CameraPublisher image_pub;

    tSdkCameraCapbility camera_capability;
    tSdkImageResolution image_resolution;
    // 相机发布
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager;
    sensor_msgs::msg::CameraInfo camera_info_msg;

    rclcpp::AsyncParametersClient::SharedPtr param_client;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_event_sub;

    int fail_cnt = 0;
    bool rotate_180 = false;
    std::atomic<bool> grab_on = false;
    std::atomic<bool> monitor_on = false;
    std::atomic<bool> camera_failed = false;
    std::atomic<bool> param_changed = false;
    // 只在模式切换的时候更改，避免频繁读写
    void declare_params();
    void init_camera();
    void reset();
    void open_device();
    void close_device();
    void start_grab();
    void stop_grab();
    void set_mind_params();
    void set_grab_params(int offset_x, int offset_y, int roi_width, int roi_height);
    void grab();
    void monitor();
    std::pair<int, int> get_sensor_height_width();
};

class CameraException : public std::exception {
   public:
    std::string info;
    CameraException(const std::string&& _info) : info{_info} {}
    const char* what() const noexcept { return info.c_str(); }
};

}  // namespace mind_camera

#endif