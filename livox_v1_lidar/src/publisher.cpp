#include "livox_v1_lidar/publisher.hpp"

using namespace livox_v1_lidar;
//LidarPublisher这个类接受雷达数据，然后发布到ROS2的PointCloud2话题里
void LidarPublisher::recv_spin()//recv_spin函数负责从 LiDAR 设备接收数据使用 Boost.Asio 库进行异步 UDP 数据接收
{
    std::array<unsigned char, pc_msg_size> recv_buf;
    boost::system::error_code error;
    std::size_t recv_length = 0;
    while (rclcpp::ok() && socket->is_open()) {//数据是否到达
        socket->async_receive(boost::asio::buffer(recv_buf),
            [&](const boost::system::error_code& error_, std::size_t length_) {
                error = error_;
                recv_length = length_;
            });
        ctx.restart();
        ctx.run_for(std::chrono::milliseconds(get_parameter("timeout_ms").as_int()));
        if (!ctx.stopped()) {
            socket->cancel();
            ctx.run();
        }
        if (error && error != boost::asio::error::message_size) {
            RCLCPP_ERROR(get_logger(), "Receiver error: %s", error.message().c_str());
            need_reconnect = true;
            continue;
        }
        //解析数据头
        auto header = reinterpret_cast<protocal::data_header*>(recv_buf.data());
        if (!check_header(*header)) {
            continue;
        }
        auto data = reinterpret_cast<protocal::type2_span*>(recv_buf.data() + sizeof(protocal::data_header));
        process_type2(*header, *data);
    }
}

void LidarPublisher::heartbeat_spin()//继续心跳和雷达设备连接
{
    std::array<uint8_t, 256> buf;
    using boost::asio::buffer;

    while (rclcpp::ok() && socket->is_open()) {
        if (need_reconnect) {
            // 发送握手包
            protocal::handshake_data hs;
            std::copy(local_ip.to_bytes().begin(), local_ip.to_bytes().end(), hs.user_ip);
            hs.data_port = local_port;
            hs.cmd_port = local_port;
            hs.imu_port = local_port;
            socket->send_to(
                buffer(buf, protocal::write_frame_buffer(buf.data(), reinterpret_cast<const uint8_t*>(&hs), sizeof(hs), buf.size())),
                boost::asio::ip::udp::endpoint(dest_ip, protocal::dest_port));

            socket->send_to(
                buffer(buf, protocal::write_frame_buffer(buf.data(), protocal::start_data, sizeof(protocal::start_data), buf.size())),
                boost::asio::ip::udp::endpoint(dest_ip, protocal::dest_port));
            need_reconnect = false;
        }
        socket->send_to(
            buffer(protocal::heartbeat_raw),
            boost::asio::ip::udp::endpoint(dest_ip, protocal::dest_port));
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

LidarPublisher::LidarPublisher(const rclcpp::NodeOptions& options) : rclcpp::Node("lidar_v1", options)
{
    declare_parameter("batch_dot_num", 9600);//每批处理的点数
    declare_parameter("local_ip", "192.168.1.50");//本地ip地址
    declare_parameter("dest_ip", "192.168.1.127");//雷达ip地址,最后1xx替换为激光雷达SN码最后两位
    declare_parameter("udp_port", 50010);//UDP端口

    // frame_id 由名命名空间决定
    auto ns = std::string_view(get_namespace());
    auto ns_pos = ns.rfind('/');
    if (ns_pos != std::string_view::npos && ns_pos + 1 < ns.size()) {
        frame_id = ns.substr(ns.rfind('/') + 1);
        frame_id.append("_frame");
    } else {
        frame_id = "default_lidar_frame";
    }
    RCLCPP_INFO(get_logger(), "frame_id: %s", frame_id.c_str());//根据命名空间生成frame_id，用于识别雷达数据的坐标系。。。？
    batch_dot_num = get_parameter("batch_dot_num").as_int();
    local_ip = boost::asio::ip::make_address_v4(get_parameter("local_ip").as_string());
    dest_ip = boost::asio::ip::make_address_v4(get_parameter("dest_ip").as_string());

    RCLCPP_INFO(get_logger(), "Params: pc_batch_size: %lu, local_ip: %s, dest_ip: %s",
        batch_dot_num, local_ip.to_string().c_str(), dest_ip.to_string().c_str());

    // 初始化了PointCloud2 话题的发布器 pc_pub
    pc_pub = create_publisher<PointCloud2>("pc_raw", rclcpp::QoS(rclcpp::KeepLast(10)));
    pc2_init();//并调用了 pc2_init 函数来进一步初始化点云数据
    declare_parameter("timeout_ms", 1000);
    //创建了一个 UDP 套接字，并启动了 heartbeat_spin 和 recv_spin 两个线程，分别用于发送心跳包和接收数据
    local_port = get_parameter("udp_port").as_int();
    int timeout_ms = get_parameter("timeout_ms").as_int();
    RCLCPP_INFO(get_logger(), "Params: udp_port: %d, timeout_ms: %d", local_port, timeout_ms);

    socket.emplace(ctx, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), local_port));
    heartbeat_thread = std::thread(&LidarPublisher::heartbeat_spin, this);
    recv_thread = std::thread(&LidarPublisher::recv_spin, this);
}

#include "rclcpp_components/register_node_macro.hpp"
//使用 RCLCPP_COMPONENTS_REGISTER_NODE 宏将 LidarPublisher 类注册为一个 ROS 2 组件节点。。。暂时没理解含义
RCLCPP_COMPONENTS_REGISTER_NODE(livox_v1_lidar::LidarPublisher)
