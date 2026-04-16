// =============================================================================
// object_marker_publisher.cpp — ROS 2 port (minimal marker broadcaster)
// =============================================================================
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class ObjectMarkerPublisher : public rclcpp::Node
{
public:
  ObjectMarkerPublisher()
  : Node("object_marker_publisher")
  {
    declare_parameter("marker_frame", "world");
    frame_ = get_parameter("marker_frame").as_string();

    pub_ = create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { publish(); });
  }

private:
  void publish()
  {
    auto m = visualization_msgs::msg::Marker();
    m.header.stamp = now();
    m.header.frame_id = frame_;
    m.ns = "object";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::CUBE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = 1.0;
    m.pose.position.y = 0.0;
    m.pose.position.z = 0.5;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.2;
    m.color.g = 1.0f;
    m.color.a = 1.0f;
    pub_->publish(m);
  }

  std::string frame_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}
