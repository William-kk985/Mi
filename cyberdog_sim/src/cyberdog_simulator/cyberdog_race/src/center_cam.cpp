// center_cam.cpp — center 模组相机桥接节点 (2026-08-14)
// 走官方 camera_api (OpenCamera+StartStream), 发布 <ns>/image_center
//   设备树 4 相机: 0=bottomleft RGB / 1=bottomright mono / 2=centerleft / 3=centerright
// 参数: cam_id(默认2) width height sync(默认true, 实测sync=true曾成功出帧)
// 用法(NX):
//   ros2 run cyberdog_race center_cam --ros-args -r __ns:=/mi_desktop_48_b0_2d_7b_02_c7 \
//       -p cam_id:=2 -p width:=1280 -p height:=800
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "camera_api/camera_api.hpp"

class CenterCamNode : public rclcpp::Node {
public:
    CenterCamNode() : Node("center_cam") {
        cam_id_ = declare_parameter("cam_id", 2);
        int w   = declare_parameter("width", 640);
        int h   = declare_parameter("height", 480);
        bool sync = declare_parameter("sync", true);
        pub_ = create_publisher<sensor_msgs::msg::Image>("image_center", 10);

        int status = 0;
        handle_ = cyberdog::camera::OpenCamera(cam_id_, status, sync);
        if (!handle_ || status != 0) {
            RCLCPP_ERROR(get_logger(), "OpenCamera(%d) FAILED status=%d", cam_id_, status);
            return;
        }
        RCLCPP_INFO(get_logger(), "OpenCamera(%d) OK status=%d sync=%d", cam_id_, status, (int)sync);
        int r = cyberdog::camera::StartStream(handle_, cyberdog::camera::kImageFormatBGR,
                                              w, h, &CenterCamNode::frame_cb, this);
        RCLCPP_INFO(get_logger(), "StartStream(%dx%d BGR) ret=%d (0=成功)", w, h, r);
    }

    ~CenterCamNode() override {
        if (handle_) {
            cyberdog::camera::StopStream(handle_);
            cyberdog::camera::CloseCamera(handle_);
        }
    }

    static int frame_cb(cv::Mat& frame, uint64_t /*ts*/, uint32_t /*capture_id*/, void* args) {
        auto* self = static_cast<CenterCamNode*>(args);
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = self->now();
        msg->header.frame_id = "center_cam";
        self->pub_->publish(*msg);
        return 0;
    }

private:
    int cam_id_{2};
    cyberdog::camera::CameraHandle handle_{nullptr};
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CenterCamNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
