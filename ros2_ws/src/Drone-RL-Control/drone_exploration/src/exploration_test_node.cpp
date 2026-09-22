#include <chrono>
#include <thread>
#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"

// To do: create an initial simple drone program that takes off, goes some distance, and land successfully
// Second To do: add services for specific tasks
class AutonomousExplorationTestNode : public rclcpp::Node {
public:
    AutonomousExplorationTestNode() 
    : Node("exploration"),
      state_(MissionState::TAKEOFF),
      state_start_time_(this->now())
    {
        takeoff_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "/simple_drone/takeoff", 10);

        land_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
        "/simple_drone/land", 10);

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/simple_drone/cmd_vel", 10);

        // Run control loop at 10Hz
        timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&AutonomousExplorationTestNode::control_callback,this));

        RCLCPP_INFO(this->get_logger(), "Exploration node started");
    }

private:

    enum class MissionState {
        TAKEOFF,
        MOVE_FORWARD,
        TURN,
        STOP,
        LAND,
        FINISHED
    };

    void control_callback() {
        auto cmd = geometry_msgs::msg::Twist();

        double elapsed = (this->now() - state_start_time_).seconds();

        switch (state_) 
        {
            case MissionState::TAKEOFF:

                publish_takeoff();

                if (elapsed > 3.0)
                {
                    change_state(MissionState::MOVE_FORWARD);
                }
                break;

            case MissionState::MOVE_FORWARD:

                // Continously published at 10 Hz
                cmd.linear.x = 0.5;

                if (elapsed > 3.0)
                {
                    change_state(MissionState::TURN);
                }
                break;

            case MissionState::TURN:

                cmd.angular.z = 0.5;

                if (elapsed > 3.0)
                {
                    change_state(MissionState::STOP);
                }
                break;

            case MissionState::STOP:

                // cmd is already zero

                if (elapsed > 1.0)
                {
                    change_state(MissionState::LAND);
                }
                break;

            case MissionState::LAND:

                publish_land();

                if (elapsed > 2.0)
                {
                    change_state(MissionState::FINISHED);
                }
                break;

            case MissionState::FINISHED:

                // Keep sending zero velocity

                RCLCPP_INFO(this->get_logger(), "Mission finished. Shutting down...");

                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                cmd_vel_publisher_->publish(cmd);

                timer_->cancel();
                rclcpp::shutdown();

                return;
        }
        // Publish velocity every timer cycle
        cmd_vel_publisher_->publish(cmd);
    }

    void change_state(MissionState new_state) {
        state_ = new_state;
        state_start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Mission state changed");
    }

    void publish_takeoff() {
        std_msgs::msg::Empty msg;
        takeoff_publisher_->publish(msg);
    }

    void publish_land() {
        std_msgs::msg::Empty msg;
        land_publisher_->publish(msg);
    }
   
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeoff_publisher_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr land_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    MissionState state_;
    rclcpp::Time state_start_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousExplorationTestNode>();

    rclcpp::spin(node);
    //rclcpp::shutdown();
    return 0;
}