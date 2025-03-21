#include <CameraApi.h>
#include <mind_camera.h>
// #include <MindCamera.h>
// #include <rm_utils/common.h>
// #include <rm_utils/frame_info.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <iostream>
// #include <rm_utils/perf.hpp>
#include <string>

// 尝试func, 如果返回值不是CAMERA_STATUS_SUCCESS(即0)则调用logger记录WARN日志
#define UPDBW(func)                                                                              \
    nRet = func;                                                                                 \
    if (nRet != CAMERA_STATUS_SUCCESS) {                                                         \
        RCLCPP_WARN(this->get_logger(), #func " failed!, error code: %d", nRet);                 \
    }

// 尝试func, 如果返回值不是CAMERA_STATUS_SUCCESS(即0)则调用logger记录FATAL日志
#define UPDBF(func)                                                                               \
    nRet = func;                                                                                  \
    if (nRet != CAMERA_STATUS_SUCCESS) {                                                          \
        RCLCPP_FATAL(this->get_logger(), #func " failed!, error code: %d", nRet);                 \
    }

// 对于不可恢复性错误重启相机节点
#define UPDBE(func)                          \
    UPDBF(func)                              \
    if (nRet != CAMERA_STATUS_SUCCESS) {     \
        reset();                             \
    }

using namespace mind_camera;

MindCameraNode::MindCameraNode(const rclcpp::NodeOptions& options) : Node("mind_camera", options) {
    RCLCPP_INFO(this->get_logger(), "MindCameraNode开启中!");
    declare_params();
    bool use_sensor_data_qos = get_parameter("use_sensor_data_qos").as_bool();
    std::string camera_info_url = get_parameter("camera_info_url").as_string();
    // 创建pub
    bool use_intra = options.use_intra_process_comms();
    if (!use_intra) {
        RCLCPP_WARN(get_logger(), "Not In Intra Process Mode");
    }
    if (!use_sensor_data_qos) {
        RCLCPP_WARN(get_logger(), "Not Use Sensor Data Qos");
    }
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    image_pub = image_transport::create_camera_publisher(this, "image", qos);

    // frame_id 由名命名空间决定
    auto ns = std::string_view(get_namespace());
    auto ns_pos = ns.rfind('/');
    if (ns_pos != std::string_view::npos && ns_pos + 1 < ns.size()) {
        frame_id = ns.substr(ns.rfind('/') + 1);
        frame_id.append("_frame");
    } else {
        frame_id = "default_camera_frame";
    }

    // load camera info
    camera_info_manager =
        std::make_unique<camera_info_manager::CameraInfoManager>(this);
    if (camera_info_manager->validateURL(camera_info_url)) {
        camera_info_manager->loadCameraInfo(camera_info_url);
        camera_info_msg = camera_info_manager->getCameraInfo();
    } else {
        RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s",
                    camera_info_url.c_str());
    }
    
    UPDBW(CameraSdkInit(1)); 

    init_camera();
    RCLCPP_WARN(get_logger(), "Starting Camera Monitor thread.");
    monitor_on = true;
    monitor_thread = std::thread(&MindCameraNode::monitor, this);

    param_client = std::make_shared<rclcpp::AsyncParametersClient>(
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_graph_interface(),
        this->get_node_services_interface());
    param_event_sub = param_client->on_parameter_event([this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
        for (auto & new_param : event->new_parameters) {
            RCLCPP_INFO(this->get_logger(), "Param %s changed.", new_param.name.c_str());
        }
        param_changed = true;
    });
};

MindCameraNode::~MindCameraNode() {
    monitor_on = false;
    if (monitor_thread.joinable()) {
        monitor_thread.join();
    }
    close_device();
};

void MindCameraNode::declare_params() {
    this->declare_parameter("sn", "");
    // https://github.com/ros-perception/image_common/blob/136807edb7ff13452214a296fb4819bc63b5b09e/image_transport/src/camera_common.cpp#L62
    this->declare_parameter("camera_info_url", "package://mind_camera/config/camera_info.yaml");
    this->declare_parameter("exposure_time", 4000.0);
    this->declare_parameter("gain", 15.0);
    this->declare_parameter("digital_shift", 6.0);
    this->declare_parameter("frame_rate", 60.0);
    this->declare_parameter("use_sensor_data_qos", false);

  // 添加白平衡相关参数
    this->declare_parameter("wb_mode", "auto");    // 白平衡模式: "auto", "once", "manual"
    this->declare_parameter("wb_rgain", 100);      // 手动模式下的红色增益
    this->declare_parameter("wb_ggain", 100);      // 手动模式下的绿色增益
    this->declare_parameter("wb_bgain", 100);      // 手动模式下的蓝色增益
    this->declare_parameter("wb_once_trigger", false); // 一键白平衡触发器

    rotate_180 = this->declare_parameter("rotate_180", false);
    if (rotate_180) {
        RCLCPP_WARN(this->get_logger(), "Rotate 180 degree");
    }
}

void MindCameraNode::init_camera() {
    tSdkCameraDevInfo camera_list[16];
    INT camera_count = 16;
    bool device_found = false;
    while (!device_found && rclcpp::ok()) {
        UPDBW(CameraEnumerateDevice(camera_list, &camera_count));
        std::string sn_to_find = get_parameter("sn").as_string();
        
        if (camera_count > 0) {
            if (get_parameter("sn").as_string() == "") {
    
                RCLCPP_WARN(this->get_logger(), "没有设置相机SN，选择第一个相机设备"); 
                UPDBE(CameraInit(&camera_list[0], -1, -1, &camera_handle));
                device_found = true;
                RCLCPP_INFO(this->get_logger(), "Camera SN: %s", camera_list[0].acSn);
            } else {
                for (int i = 0; i < camera_count; ++i) {
                    if (std::strncmp(camera_list[i].acSn, sn_to_find.c_str(), 64U) == 0) {
                        UPDBE(CameraInit(&camera_list[i], -1, -1, &camera_handle));
                        device_found = true;
                        RCLCPP_INFO(this->get_logger(), "Camera SN: %s", camera_list[i].acSn);
                        break;
                    }
                }
            }
        }
        if (device_found) {
            break;
        } else {
            RCLCPP_WARN(this->get_logger(), "Camera SN: %s not found.", sn_to_find.c_str());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    if (device_found) {
        UPDBW(CameraGetCapability(camera_handle, &camera_capability));
        open_device();
        set_mind_params();
        start_grab();
    }
}

void MindCameraNode::monitor() {
    while (rclcpp::ok() && monitor_on) {
        if (camera_failed) {
            RCLCPP_ERROR(this->get_logger(), "获取相机失败，重启中......");
            reset();
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

void MindCameraNode::start_grab() {
    UPDBW(CameraSetTriggerMode(camera_handle, 0));// 设置为连续采集模式
    UPDBW(CameraPlay(camera_handle));// 开始采集
    grab_on = true; // 开启采集线程
    camera_failed = false;
    capture_thread = std::thread(&MindCameraNode::grab, this);
}

void MindCameraNode::stop_grab() {
    grab_on = false;
    if (capture_thread.joinable()) {
        capture_thread.join();
    }
    if (camera_handle) {
        CameraStop(camera_handle);
    }
}

std::pair<int, int> MindCameraNode::get_sensor_height_width() {
    return std::pair{(int)camera_capability.sResolutionRange.iHeightMax, 
                    (int)camera_capability.sResolutionRange.iWidthMax};
}

void MindCameraNode::set_mind_params() {
    static bool first_set = true;
    static bool wb_once_last = false;

    if (first_set) {
        first_set = false;
        UPDBW(CameraSetAeState(camera_handle, FALSE)); 

        // int wbRet = CameraSetWbMode(camera_handle, TRUE);
        // if (wbRet != CAMERA_STATUS_SUCCESS) {
        //     RCLCPP_WARN(this->get_logger(), "相机不支持自动白平衡模式, error: %d, 使用手动白平衡", wbRet);
        //     CameraSetWbMode(camera_handle, FALSE);
        // } else {
        //     CameraSetOnceWB(camera_handle);
        // }
            // 从参数中获取白平衡模式
    std::string wb_mode = get_parameter("wb_mode").as_string();
    bool wb_once_trigger = get_parameter("wb_once_trigger").as_bool();
    
    if (wb_mode == "auto") { // 处理白平衡设置
        // 尝试设置自动白平衡
        int wbRet = CameraSetWbMode(camera_handle, TRUE);
        if (wbRet != CAMERA_STATUS_SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "相机不支持自动白平衡模式, error: %d, 切换到手动白平衡", wbRet);
            wb_mode = "manual"; // 自动失败，切换到手动模式
        } else {
            RCLCPP_INFO(this->get_logger(), "自动白平衡模式已启用");
        }
    }
    
    if (wb_mode == "manual") {
        // 设置手动白平衡，并使用参数中的RGB增益值
        CameraSetWbMode(camera_handle, FALSE);
        int r_gain = get_parameter("wb_rgain").as_int();
        int g_gain = get_parameter("wb_ggain").as_int();
        int b_gain = get_parameter("wb_bgain").as_int();
        
        // 设置RGB增益
        int rgbRet = CameraSetUserClrTempGain(camera_handle, r_gain, g_gain, b_gain);
        if (rgbRet == CAMERA_STATUS_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "手动白平衡已设置: R=%d, G=%d, B=%d", r_gain, g_gain, b_gain);
        } else {
            RCLCPP_WARN(this->get_logger(), "设置手动白平衡失败, error: %d", rgbRet);
        }
    }
    
    if (wb_mode == "once" || (wb_once_trigger && !wb_once_last)) {// 处理一键白平衡触发
        CameraSetWbMode(camera_handle, FALSE); // 切换到手动模式
        int onceRet = CameraSetOnceWB(camera_handle); // 执行一次白平衡
        if (onceRet == CAMERA_STATUS_SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "已执行一键白平衡");
            
          
            if (wb_once_trigger) {
                rclcpp::Parameter param("wb_once_trigger", false);  // 如果是通过参数触发的一键白平衡，重置触发器
                std::vector<rclcpp::Parameter> params = {param};
                this->set_parameters(params);
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "执行一键白平衡失败, error: %d", onceRet);
        }
    }
    wb_once_last = wb_once_trigger;

        UPDBW(CameraGetImageResolution(camera_handle, &image_resolution));
        UPDBW(CameraSetTriggerMode(camera_handle, 0));
        UPDBW(CameraSetIspOutFormat(camera_handle, CAMERA_MEDIA_TYPE_BGR8));
    }

    UPDBW(CameraSetFrameSpeed(camera_handle, FRAME_SPEED_NORMAL)); // 相机帧率调整

    // UPDBW(CameraSetFrameRate(camera_handle, (int)get_parameter("frame_rate").as_double()));
    // int frameRateRet = CameraSetFrameRate(camera_handle, (int)get_parameter("frame_rate").as_double());
    // if (frameRateRet != CAMERA_STATUS_SUCCESS) {
    //     RCLCPP_WARN(this->get_logger(), "相机不支持设置帧率, error: %d", frameRateRet);
    // }

    UPDBW(CameraSetExposureTime(camera_handle, get_parameter("exposure_time").as_double()));
    UPDBW(CameraSetAnalogGain(camera_handle, (int)get_parameter("gain").as_double()));
}

void MindCameraNode::grab() {
    tSdkFrameHead frame_info;
    BYTE* image_buffer;
    
    RCLCPP_INFO(this->get_logger(), "已发布图像！");

    sensor_msgs::msg::Image image_msg;
    image_msg.encoding = "bgr8";
    image_msg.height = image_resolution.iHeight;
    image_msg.width = image_resolution.iWidth;
    image_msg.step = image_resolution.iWidth * 3;
    image_msg.data.resize(image_resolution.iWidth * image_resolution.iHeight * 3);

    while (rclcpp::ok() && grab_on) {
        if (param_changed) {
            set_mind_params();
            param_changed = false;
        }
        
        nRet = CameraGetImageBuffer(camera_handle, &frame_info, &image_buffer, 1000);
        if (CAMERA_STATUS_SUCCESS == nRet) {  // 将图像数据转换成RGB8格式
          
            CameraImageProcess(camera_handle, image_buffer, image_msg.data.data(), &frame_info);

            if (rotate_180) {
                for (unsigned i = 0; i < image_resolution.iWidth * image_resolution.iHeight / 2; i++) {
                    std::swap(image_msg.data[i * 3], image_msg.data[(image_resolution.iWidth * image_resolution.iHeight - 1 - i) * 3]);
                    std::swap(image_msg.data[i * 3 + 1], image_msg.data[(image_resolution.iWidth * image_resolution.iHeight - 1 - i) * 3 + 1]);
                    std::swap(image_msg.data[i * 3 + 2], image_msg.data[(image_resolution.iWidth * image_resolution.iHeight - 1 - i) * 3 + 2]);
                }
            }

            auto time_now = this->now();
            image_msg.header.stamp = this->now();
            image_msg.header.frame_id = frame_id;//帧ID在这里
            camera_info_msg.header.stamp = time_now;
            camera_info_msg.header.frame_id = frame_id;//这个是设置相机信息帧ID

              RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000, 
                "相机发布图像，frame_id: %s", frame_id.c_str());

            image_pub.publish(image_msg, camera_info_msg);

            // 释放图像缓冲区hh
            CameraReleaseImageBuffer(camera_handle, image_buffer);
            fail_cnt = 0;
        } else {
            RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%d]", nRet);
            fail_cnt++;
        }

        if (fail_cnt > 5) {
            RCLCPP_FATAL(this->get_logger(), "获取相机失败！");
            grab_on = false;
            camera_failed = true;
        }
    }
}

void MindCameraNode::reset() {
    close_device();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    init_camera();
}

void MindCameraNode::open_device() {
    //通过连续的停止-开始来确保相机状态正常
    UPDBE(CameraStop(camera_handle));
    UPDBE(CameraPlay(camera_handle));
}

void MindCameraNode::close_device() {
    stop_grab();
    if (camera_handle) {
        CameraUnInit(camera_handle);
        camera_handle = 0;
    }
    RCLCPP_INFO(this->get_logger(), "迈德相机关闭！");
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mind_camera::MindCameraNode)

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<MindCameraNode>(rclcpp::NodeOptions());
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }