// =============================================================================
// object_frame_publisher.cpp — ROS 2 port
// Subscribes to binary masks + depth, projects to 3D via PCL, publishes TF.
// =============================================================================
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <map>
#include <sstream>
#include <iomanip>

class ObjectFramePublisher : public rclcpp::Node
{
public:
  ObjectFramePublisher()
  : Node("object_frame_publisher"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Parameters
    declare_parameter("depth_topic", "/patrick/camera/depth/image_raw");
    declare_parameter("camera_info_topic", "/patrick/camera/camera_info");
    declare_parameter("optical_frame", "camera_depth_optical_frame");
    declare_parameter("camera_link_frame", "camera_link");
    declare_parameter("pcl_mean_k", 50);
    declare_parameter("pcl_stddev_mul_thresh", 0.2);
    declare_parameter("object_names", std::vector<std::string>{"red_cylinder", "green_cylinder", "blue_box"});

    optical_frame_ = get_parameter("optical_frame").as_string();
    camera_link_frame_ = get_parameter("camera_link_frame").as_string();
    pcl_mean_k_ = get_parameter("pcl_mean_k").as_int();
    pcl_stddev_ = get_parameter("pcl_stddev_mul_thresh").as_double();

    auto depth_topic = get_parameter("depth_topic").as_string();
    auto info_topic = get_parameter("camera_info_topic").as_string();
    auto object_names = get_parameter("object_names").as_string_array();

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, 1, [this](sensor_msgs::msg::Image::SharedPtr msg) {
        latest_depth_ = msg; has_depth_ = true;
      });

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, 1, [this](sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        fx_ = msg->k[0]; fy_ = msg->k[4]; cx_ = msg->k[2]; cy_ = msg->k[5];
        has_info_ = true;
      });

    detected_pub_ = create_publisher<std_msgs::msg::String>("/detected_objects", 1);

    for (const auto & name : object_names) {
      std::string topic = "/detected_" + name + "_mask";
      auto sub = create_subscription<sensor_msgs::msg::Image>(
        topic, 1,
        [this, name](sensor_msgs::msg::Image::SharedPtr msg) {
          mask_callback(msg, name);
        });
      mask_subs_.push_back(sub);
      RCLCPP_INFO(get_logger(), "Subscribed to %s -> TF: %s", topic.c_str(), name.c_str());
    }
  }

private:
  float read_depth(const cv::Mat & depth, int u, int v)
  {
    float d = 0.0f;
    if (depth.type() == CV_32FC1) d = depth.at<float>(v, u);
    else if (depth.type() == CV_16UC1) d = depth.at<uint16_t>(v, u) / 1000.0f;
    if (std::isnan(d) || std::isinf(d) || d <= 0.0f) return 0.0f;
    return d;
  }

  void mask_callback(const sensor_msgs::msg::Image::SharedPtr msg, const std::string & name)
  {
    if (!has_info_ || !has_depth_) return;

    auto mask_cv = cv_bridge::toCvShare(msg, "mono8");
    auto depth_cv = cv_bridge::toCvShare(latest_depth_);
    const cv::Mat & mask = mask_cv->image;
    const cv::Mat & depth = depth_cv->image;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->reserve(cv::countNonZero(mask));

    for (int r = 0; r < depth.rows; ++r) {
      for (int c = 0; c < depth.cols; ++c) {
        if (r >= mask.rows || c >= mask.cols) continue;
        if (mask.at<uchar>(r, c) == 0) continue;
        float d = read_depth(depth, c, r);
        if (d <= 0.0f) continue;
        pcl::PointXYZ pt;
        pt.z = d;
        pt.x = (c - cx_) * d / fx_;
        pt.y = (r - cy_) * d / fy_;
        cloud->push_back(pt);
      }
    }

    if (cloud->size() < 10) return;

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(pcl_mean_k_);
    sor.setStddevMulThresh(pcl_stddev_);
    sor.filter(*cloud);
    if (cloud->empty()) return;

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    // Publish TF
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = optical_frame_;
    tf.child_frame_id = name;
    tf.transform.translation.x = centroid[0];
    tf.transform.translation.y = centroid[1];
    tf.transform.translation.z = centroid[2];
    tf.transform.rotation.w = 1.0;
    tf_broadcaster_.sendTransform(tf);

    // Transform to camera_link frame
    geometry_msgs::msg::PointStamped pt_opt;
    pt_opt.header.stamp = now();
    pt_opt.header.frame_id = optical_frame_;
    pt_opt.point.x = centroid[0];
    pt_opt.point.y = centroid[1];
    pt_opt.point.z = centroid[2];

    geometry_msgs::msg::PointStamped pt_cam;
    try {
      auto tf_to_cam = tf_buffer_.lookupTransform(
        camera_link_frame_, optical_frame_, tf2::TimePointZero, tf2::durationFromSec(0.1));
      tf2::doTransform(pt_opt, pt_cam, tf_to_cam);
    } catch (tf2::TransformException &) {
      pt_cam.point.x = centroid[2];
      pt_cam.point.y = -centroid[0];
      pt_cam.point.z = -centroid[1];
    }

    positions_[name] = pt_cam.point;

    // Bounding box
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);
    geometry_msgs::msg::Vector3 dims;
    dims.x = max_pt.x - min_pt.x;
    dims.y = max_pt.y - min_pt.y;
    dims.z = max_pt.z - min_pt.z;
    dimensions_[name] = dims;

    publish_detected();
  }

  void publish_detected()
  {
    std::ostringstream ss;
    for (const auto & [name, pos] : positions_) {
      ss << "object: " << name
         << " x:" << std::fixed << std::setprecision(3) << pos.x
         << " y:" << pos.y << " z:" << pos.z;
      auto dit = dimensions_.find(name);
      if (dit != dimensions_.end())
        ss << " w:" << dit->second.x << " h:" << dit->second.y << " d:" << dit->second.z;
      ss << "\n";
    }
    std_msgs::msg::String out;
    out.data = ss.str();
    detected_pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> mask_subs_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detected_pub_;

  tf2_ros::TransformBroadcaster tf_broadcaster_{this};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  sensor_msgs::msg::Image::SharedPtr latest_depth_;
  std::map<std::string, geometry_msgs::msg::Point> positions_;
  std::map<std::string, geometry_msgs::msg::Vector3> dimensions_;

  std::string optical_frame_, camera_link_frame_;
  double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
  int pcl_mean_k_ = 50;
  double pcl_stddev_ = 0.2;
  bool has_info_ = false, has_depth_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectFramePublisher>());
  rclcpp::shutdown();
  return 0;
}
