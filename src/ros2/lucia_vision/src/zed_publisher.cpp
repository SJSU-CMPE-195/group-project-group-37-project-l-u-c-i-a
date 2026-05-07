/**
 * zed_publisher.cpp
 *
 * Minimal ROS2 node for the ZED 2i camera on the Jetson Nano.
 * Publishes compressed RGB, visual-inertial odometry, and 3D object detections.
 *
 * Topics published:
 *   /zed/rgb/image/compressed  (sensor_msgs/CompressedImage)  — JPEG, 30 Hz
 *   /zed/odom                  (nav_msgs/Odometry)             — 30 Hz
 *   /zed/objects               (geometry_msgs/PoseArray)       — 30 Hz
 *
 * Run inside dustynv Docker container with ZED SDK and CUDA mounted:
 *   docker run --runtime nvidia --network host --privileged \
 *     -v /usr/local/zed:/usr/local/zed \
 *     -v /usr/local/cuda:/usr/local/cuda \
 *     -v ~/ros2_ws:/ros2_ws \
 *     dustynv/ros:humble-ros-base-l4t-r32.7.1 bash
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

class ZedPublisher : public rclcpp::Node
{
public:
  ZedPublisher() : Node("zed_publisher")
  {
    rgb_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      "/zed/rgb/image/compressed", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      "/zed/odom", 10);
    objects_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "/zed/objects", 10);

    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION::HD720;
    init_params.camera_fps = 30;
    init_params.coordinate_units = sl::UNIT::METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    // PERFORMANCE depth needed for 3D object positions and visual odometry
    init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE;

    auto status = cam_.open(init_params);
    if (status != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Failed to open ZED camera: %s",
        sl::toString(status).c_str());
      rclcpp::shutdown();
      return;
    }

    sl::PositionalTrackingParameters pt_params;
    cam_.enablePositionalTracking(pt_params);

    sl::ObjectDetectionParameters od_params;
    od_params.enable_tracking = true;
    od_params.detection_model = sl::DETECTION_MODEL::MULTI_CLASS_BOX_FAST;
    cam_.enableObjectDetection(od_params);

    RCLCPP_INFO(get_logger(), "ZED 2i opened — HD720 @ 30fps");

    timer_ = create_wall_timer(33ms, std::bind(&ZedPublisher::publish_loop, this));
  }

  ~ZedPublisher()
  {
    cam_.disableObjectDetection();
    cam_.disablePositionalTracking();
    cam_.close();
  }

private:
  void publish_loop()
  {
    if (cam_.grab() != sl::ERROR_CODE::SUCCESS) return;

    auto now = get_clock()->now();

    publish_rgb(now);
    publish_odom(now);
    publish_objects(now);
  }

  void publish_rgb(const rclcpp::Time & stamp)
  {
    sl::Mat image;
    cam_.retrieveImage(image, sl::VIEW::LEFT);

    cv::Mat cv_bgra(image.getHeight(), image.getWidth(), CV_8UC4,
      image.getPtr<sl::uchar1>());
    cv::Mat cv_bgr;
    cv::cvtColor(cv_bgra, cv_bgr, cv::COLOR_BGRA2BGR);

    std::vector<uchar> buf;
    cv::imencode(".jpg", cv_bgr, buf, {cv::IMWRITE_JPEG_QUALITY, 80});

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "zed_camera_link";
    msg.format = "jpeg";
    msg.data = buf;
    rgb_pub_->publish(msg);
  }

  void publish_odom(const rclcpp::Time & stamp)
  {
    sl::Pose pose;
    auto state = cam_.getPosition(pose, sl::REFERENCE_FRAME::WORLD);
    if (state != sl::POSITIONAL_TRACKING_STATE::OK) return;

    auto t = pose.getTranslation();
    auto q = pose.getOrientation();

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";
    msg.pose.pose.position.x = t.x;
    msg.pose.pose.position.y = t.y;
    msg.pose.pose.position.z = t.z;
    msg.pose.pose.orientation.x = q.ox;
    msg.pose.pose.orientation.y = q.oy;
    msg.pose.pose.orientation.z = q.oz;
    msg.pose.pose.orientation.w = q.ow;
    odom_pub_->publish(msg);
  }

  void publish_objects(const rclcpp::Time & stamp)
  {
    sl::Objects objects;
    sl::ObjectDetectionRuntimeParameters od_rt;
    od_rt.detection_confidence_threshold = 50;
    cam_.retrieveObjects(objects, od_rt);

    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "zed_camera_link";

    for (auto & obj : objects.object_list) {
      if (obj.tracking_state != sl::OBJECT_TRACKING_STATE::OK) continue;
      geometry_msgs::msg::Pose p;
      p.position.x = obj.position.x;
      p.position.y = obj.position.y;
      p.position.z = obj.position.z;
      p.orientation.w = 1.0;
      msg.poses.push_back(p);
    }

    objects_pub_->publish(msg);
  }

  sl::Camera cam_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr objects_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZedPublisher>());
  rclcpp::shutdown();
  return 0;
}
