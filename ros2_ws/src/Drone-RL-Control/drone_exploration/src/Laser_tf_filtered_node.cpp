#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/node_options.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "message_filters/subscriber.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"

#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

class LaserTfFilteredNode : public rclcpp::Node
{
public:
  LaserTfFilteredNode()
: Node("laser_tf_filtered_node",
       rclcpp::NodeOptions().parameter_overrides(
         {rclcpp::Parameter("use_sim_time", true)}))
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
        this->get_clock(),
        tf2::durationFromSec(30.0));

    auto timer_interface =
      std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface());

    tf_buffer_->setCreateTimerInterface(timer_interface);

    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/simple_drone/laser_scanner/filtered",
        rclcpp::SensorDataQoS());

    cloud_sub_.subscribe(
      this,
      "/simple_drone/laser_scanner/out",
      rclcpp::SensorDataQoS().get_rmw_qos_profile());

    tf_filter_ =
      std::make_shared<
        tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
          cloud_sub_,
          *tf_buffer_,
          target_frame_,
          10,
          this->get_node_logging_interface(),
          this->get_node_clock_interface(),
          std::chrono::duration<int>(3));

    tf_filter_->registerCallback(
      std::bind(&LaserTfFilteredNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "Laser TF filter started: %s -> %s",
      "simple_drone/laser_link",
      target_frame_.c_str());
  }

private:

  void cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
  {
    try {

      sensor_msgs::msg::PointCloud2 transformed_cloud;

      tf2::doTransform(
        *cloud_msg,
        transformed_cloud,
        tf_buffer_->lookupTransform(
          target_frame_,
          cloud_msg->header.frame_id,
          cloud_msg->header.stamp));

      transformed_cloud.header.stamp =
        cloud_msg->header.stamp;

      transformed_cloud.header.frame_id =
        target_frame_;

      cloud_pub_->publish(transformed_cloud);

    } catch (const tf2::TransformException & ex) {

      RCLCPP_WARN(
        this->get_logger(),
        "Could not transform laser cloud: %s",
        ex.what());
    }
  }

  std::string target_frame_ =
    "simple_drone/odom";

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<
    sensor_msgs::msg::PointCloud2> cloud_sub_;

  std::shared_ptr<
    tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>
    tf_filter_;

  rclcpp::Publisher<
    sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<LaserTfFilteredNode>());

  rclcpp::shutdown();

  return 0;
}