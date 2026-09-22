#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <limits>
#include <string>
#include <vector>
#include <utility>
#include <functional>
#include <queue>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/empty.hpp"

using namespace std::chrono_literals;

class AutonomousExplorationNode : public rclcpp::Node
{
public:
    AutonomousExplorationNode()
        : Node("exploration"),
          state_(State::TAKEOFF),
          odom_received_(false),
          lidar_received_(false),
          front_obstacle_(false),
          takeoff_sent_(false),
          landing_sent_(false),
          current_x_(0.0),
          current_y_(0.0),
          current_z_(0.0),
          current_yaw_(0.0),
          left_distance_(10.0),
          right_distance_(10.0),
          front_distance_(10.0),
          path_index_(0),
          path_planned_(false),
          avoidance_direction_(1),
          avoid_target_yaw_(0.0),
          pass_start_x_(0.0),
          pass_start_y_(0.0),
          last_progress_x_(0.0),
          last_progress_y_(0.0),
          stuck_reversal_count_(0),
          max_stuck_reversals_(2),
          last_reversal_x_(0.0),
          last_reversal_y_(0.0),
          chokepoint_distance_(1.5)
    {
        // goal
        goal_x_ = -25.0;
        goal_y_ = 14.5;
        goal_z_ = 1.0;

        // publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/simple_drone/cmd_vel", 10);

        takeoff_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "/simple_drone/takeoff", 10);

        land_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "/simple_drone/land", 10);

        // odometry subscription
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/simple_drone/odom",
            10,
            std::bind(&AutonomousExplorationNode::odomCallback,
                this, std::placeholders::_1));

        // LiDAR subscription
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/simple_drone/laser_scanner/out",
            10,
            std::bind(&AutonomousExplorationNode::lidarCallback,
                this, std::placeholders::_1));

        // map subscription
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            std::bind(&AutonomousExplorationNode::mapCallback,
                this, std::placeholders::_1));

        // control timer at 10 Hz
        control_timer_ = this->create_wall_timer(
            100ms,
            std::bind(&AutonomousExplorationNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "Autonomous exploration node started.");
        RCLCPP_INFO(this->get_logger(), "Goal: x=%.2f y=%.2f z=%.2f",
            goal_x_, goal_y_, goal_z_);
        RCLCPP_INFO(this->get_logger(), "LiDAR obstacle distance: %.2f m",
            obstacle_distance_);
    }

private:

    enum class State
    {
        TAKEOFF,
        NAVIGATE,
        AVOID_OBSTACLE,
        PASS_OBSTACLE,
        LAND,
        DONE
    };

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;

        const double qx = msg->pose.pose.orientation.x;
        const double qy = msg->pose.pose.orientation.y;
        const double qz = msg->pose.pose.orientation.z;
        const double qw = msg->pose.pose.orientation.w;

        current_yaw_ = std::atan2(2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz));
        odom_received_ = true;
    }

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        lidar_received_ = true;
        double min_front = std::numeric_limits<double>::max();
        double min_left = std::numeric_limits<double>::max();
        double min_right = std::numeric_limits<double>::max();

        try {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
                const double x = *iter_x;
                const double y = *iter_y;
                const double z = *iter_z;

                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                    continue;
                }

                const double distance = std::sqrt(x * x + y * y + z * z);
                if (distance < 0.05 || distance > lidar_max_range_) {
                    continue;
                }

                if (x > 0.0 && std::abs(y) < front_width_) {
                    min_front = std::min(min_front, distance);
                }

                if (y > 0.15 && x > -0.5 && x < 2.0) {
                    min_left = std::min(min_left, distance);
                }

                if (y < -0.15 && x > -0.5 && x < 2.0) {
                    min_right = std::min(min_right, distance);
                }
            }
        }
        catch (const std::exception &e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Could not process LiDAR point cloud: %s", e.what());
            return;
        }

        left_distance_ = (min_left ==
            std::numeric_limits<double>::max()) ? lidar_max_range_ : min_left;

        right_distance_ = (min_right ==
            std::numeric_limits<double>::max()) ? lidar_max_range_ : min_right;

        front_distance_ = (min_front == std::numeric_limits<double>::max())
            ? lidar_max_range_ : min_front;

        front_obstacle_ = (min_front < obstacle_distance_);
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (!map_frame_logged_) {
            RCLCPP_INFO(this->get_logger(),
                "Static map received: %u x %u cells @ %.3f m/cell, "
                "frame '%s', origin (%.2f, %.2f)",
                msg->info.width, msg->info.height,
                msg->info.resolution, msg->header.frame_id.c_str(),
                msg->info.origin.position.x, msg->info.origin.position.y);

            map_frame_logged_ = true;
        }

        latest_map_ = msg;
        buildInflatedGrid();

        path_planned_ = false;
        planned_path_.clear();
        path_index_ = 0;
    }

    void buildInflatedGrid() {
        if (!latest_map_) {
            return;
        }

        const auto &info = latest_map_->info;
        const int width = static_cast<int>(info.width);
        const int height = static_cast<int>(info.height);

        std::vector<uint8_t> hazard(
            static_cast<size_t>(width) * static_cast<size_t>(height), 0);

        for (int i = 0; i < width * height; ++i) {
            const int8_t v = latest_map_->data[i];
            hazard[i] = (v == -1 || v >= occupied_threshold_) ? 1 : 0;
        }

        inflated_occupied_.assign(
            static_cast<size_t>(width) * static_cast<size_t>(height), 0);

        const int radius_cells = std::max(
            0,
            static_cast<int>(std::ceil(safety_radius_ / info.resolution)));

        for (int row = 0; row < height; ++row) {
            for (int col = 0; col < width; ++col) {
                if (!hazard[row * width + col]) {
                    continue;
                }

                for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
                    for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
                        if (dr * dr + dc * dc > radius_cells * radius_cells) {
                            continue;
                        }

                        const int nr = row + dr;
                        const int nc = col + dc;

                        if (nr < 0 || nc < 0 || nr >= height || nc >= width) {
                            continue;
                        }

                        inflated_occupied_[nr * width + nc] = 1;
                    }
                }
            }
        }

        RCLCPP_INFO(this->get_logger(),
            "Built inflated planning grid: %dx%d cells, safety radius "
            "%.2f m (%d cells).",
            width, height, safety_radius_, radius_cells);
    }

    bool isCellFree(double x, double y) const {
        if (!latest_map_ || inflated_occupied_.empty()) {
            return false;
        }

        const auto &info = latest_map_->info;

        const int col = static_cast<int>(
            std::floor((x - info.origin.position.x) / info.resolution));

        const int row = static_cast<int>(
            std::floor((y - info.origin.position.y) / info.resolution));

        if (col < 0 || row < 0 ||
            col >= static_cast<int>(info.width) ||
            row >= static_cast<int>(info.height)) {
            return false;
        }

        const size_t index =
            static_cast<size_t>(row) * info.width + static_cast<size_t>(col);

        return inflated_occupied_[index] == 0;
    }

    bool isPathBlocked(double x0, double y0, double x1, double y1,
                        double step = 0.2) const {
        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const double dist = std::sqrt(dx * dx + dy * dy);

        const int steps = std::max(1, static_cast<int>(dist / step));

        for (int i = 0; i <= steps; ++i) {
            const double t = static_cast<double>(i) / steps;

            if (!isCellFree(x0 + t * dx, y0 + t * dy))
            {
                return true;
            }
        }

        return false;
    }

    bool planPathAStar(double start_x, double start_y,
                        double goal_x, double goal_y,
                        std::vector<std::pair<double, double>> &out_path) {
        if (!latest_map_ || inflated_occupied_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No map loaded yet — cannot plan a path.");
            return false;
        }

        const auto &info = latest_map_->info;
        const int width = static_cast<int>(info.width);
        const int height = static_cast<int>(info.height);

        auto worldToGrid = [&](double x, double y, int &row, int &col)
        {
            col = static_cast<int>(
                std::floor((x - info.origin.position.x) / info.resolution));
            row = static_cast<int>(
                std::floor((y - info.origin.position.y) / info.resolution));
        };

        auto inBounds = [&](int r, int c)
        {
            return r >= 0 && c >= 0 && r < height && c < width;
        };

        auto isFree = [&](int r, int c)
        {
            return inBounds(r, c) &&
                   inflated_occupied_[r * width + c] == 0;
        };

        int start_row, start_col, goal_row, goal_col;
        worldToGrid(start_x, start_y, start_row, start_col);
        worldToGrid(goal_x, goal_y, goal_row, goal_col);

        if (!inBounds(start_row, start_col) || !inBounds(goal_row, goal_col)) {
            RCLCPP_ERROR(this->get_logger(),
                "Start or goal falls outside the map extent — cannot plan.");
            return false;
        }

        if (!isFree(start_row, start_col)) {
            RCLCPP_WARN(this->get_logger(),
                "Start cell is inside the inflated safety margin — attempting anyway.");
        }

        if (!isFree(goal_row, goal_col)) {
            RCLCPP_ERROR(this->get_logger(),
                "Goal cell is occupied (or within safety margin) — cannot plan route.");
            return false;
        }

        static const int dr8[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
        static const int dc8[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
        const double diag = std::sqrt(2.0);
        static const double cost8_template[8] =
            {0, 1.0, 0, 1.0, 1.0, 0, 1.0, 0};
        double cost8[8];
        for (int i = 0; i < 8; ++i) {
            cost8[i] = (dr8[i] != 0 && dc8[i] != 0) ? diag : cost8_template[i];
        }

        auto heuristic = [&](int r, int c)
        {
            const int ddr = std::abs(goal_row - r);
            const int ddc = std::abs(goal_col - c);
            return static_cast<double>(ddr + ddc) +
                   (diag - 2.0) * std::min(ddr, ddc);
        };

        auto key = [&](int r, int c) -> int64_t
        {
            return static_cast<int64_t>(r) * width + c;
        };

        std::unordered_map<int64_t, double> gScore;
        std::unordered_map<int64_t, int64_t> cameFrom;
        std::unordered_map<int64_t, bool> closed;

        using QueueItem = std::pair<double, int64_t>;
        std::priority_queue<
            QueueItem, std::vector<QueueItem>, std::greater<QueueItem>>
            open;

        const int64_t startKey = key(start_row, start_col);
        const int64_t goalKey = key(goal_row, goal_col);

        gScore[startKey] = 0.0;
        open.push({heuristic(start_row, start_col), startKey});

        bool found = false;

        while (!open.empty()) {
            const QueueItem top = open.top();
            open.pop();

            const int64_t currentKey = top.second;

            if (closed[currentKey]) {
                continue;
            }
            closed[currentKey] = true;

            if (currentKey == goalKey) {
                found = true;
                break;
            }

            const int currentRow = static_cast<int>(currentKey / width);
            const int currentCol = static_cast<int>(currentKey % width);

            for (int i = 0; i < 8; ++i) {
                const int nr = currentRow + dr8[i];
                const int nc = currentCol + dc8[i];

                if (!isFree(nr, nc)) {
                    continue;
                }

                if (dr8[i] != 0 && dc8[i] != 0) {
                    if (!isFree(currentRow + dr8[i], currentCol) ||
                        !isFree(currentRow, currentCol + dc8[i])) {
                        continue;
                    }
                }

                const int64_t neighborKey = key(nr, nc);
                const double tentativeG = gScore[currentKey] + cost8[i];

                const auto it = gScore.find(neighborKey);
                if (it == gScore.end() || tentativeG < it->second) {
                    gScore[neighborKey] = tentativeG;
                    cameFrom[neighborKey] = currentKey;

                    const double fScore = tentativeG + heuristic(nr, nc);
                    open.push({fScore, neighborKey});
                }
            }
        }

        if (!found) {
            RCLCPP_ERROR(this->get_logger(),
                "A* failed to find a path to the goal through the mapped environment.");
            return false;
        }

        std::vector<int64_t> pathKeys;
        int64_t cur = goalKey;
        pathKeys.push_back(cur);
        while (cur != startKey) {
            cur = cameFrom.at(cur);
            pathKeys.push_back(cur);
        }
        std::reverse(pathKeys.begin(), pathKeys.end());

        out_path.clear();
        for (const int64_t k : pathKeys) {
            const int r = static_cast<int>(k / width);
            const int c = static_cast<int>(k % width);

            const double wx =
                info.origin.position.x + (c + 0.5) * info.resolution;
            const double wy =
                info.origin.position.y + (r + 0.5) * info.resolution;

            out_path.emplace_back(wx, wy);
        }

        const size_t raw_count = out_path.size();
        pruneCollinearWaypoints(out_path);

        RCLCPP_INFO(this->get_logger(),
            "A* planned a path with %zu waypoints (%zu before pruning).",
            out_path.size(), raw_count);

        return true;
    }

    static void pruneCollinearWaypoints(
        std::vector<std::pair<double, double>> &path) {
        if (path.size() < 3) {
            return;
        }

        std::vector<std::pair<double, double>> pruned;
        pruned.push_back(path.front());

        for (size_t i = 1; i + 1 < path.size(); ++i) {
            const auto &prev = pruned.back();
            const auto &curr = path[i];
            const auto &next = path[i + 1];

            const double dx1 = curr.first - prev.first;
            const double dy1 = curr.second - prev.second;
            const double dx2 = next.first - curr.first;
            const double dy2 = next.second - curr.second;

            const double cross = dx1 * dy2 - dy1 * dx2;

            if (std::abs(cross) > 1e-6) {
                pruned.push_back(curr);
            }
        }

        pruned.push_back(path.back());
        path = std::move(pruned);
    }

    void controlLoop() {
        if (!odom_received_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                2000, "Waiting for odometry...");
            publishStop();
            return;
        }

        switch (state_) {
            case State::TAKEOFF:
                takeoff();
                break;

            case State::NAVIGATE:
                navigate();
                break;

            case State::AVOID_OBSTACLE:
                avoid_obstacle();
                break;

            case State::PASS_OBSTACLE:
                pass_obstacle();
                break;

            case State::LAND:
                land();
                break;

            case State::DONE:
                publishStop();
                break;
        }
    }

    void takeoff() {
        geometry_msgs::msg::Twist cmd;

        std_msgs::msg::Empty takeoff_msg;
        takeoff_pub_->publish(takeoff_msg);

        if (!takeoff_sent_) {
            RCLCPP_INFO(this->get_logger(), "TAKEOFF command sent.");
            takeoff_sent_ = true;
        }

        const double target_takeoff_z = 1.0;
        const double kp_takeoff_z = 0.8;
        const double z_error = target_takeoff_z - current_z_;

        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;

        cmd.linear.z = kp_takeoff_z * z_error;
        cmd.linear.z = std::clamp(cmd.linear.z, -0.3, 0.4);

        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;

        cmd_vel_pub_->publish(cmd);

        if (current_z_ >= 0.85) {
            RCLCPP_INFO(this->get_logger(),
                "Takeoff altitude reached: z=%.2f m", current_z_);
            publishStop();

            state_ = State::NAVIGATE;
            RCLCPP_INFO(this->get_logger(), "Starting autonomous navigation.");
        }
        else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                1000, "Taking off... altitude = %.2f m", current_z_);
        }
    }

    void navigate() {
        // Halt if map is not available yet
        if (!latest_map_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                3000, "Waiting for map on /map before planning A* path...");
            publishStop();
            return;
        }

        // Plan or replan whenever path_planned_ is false
        if (!path_planned_) {
            RCLCPP_INFO(this->get_logger(),
                "Planning A* route from (%.2f, %.2f) to goal (%.2f, %.2f)...",
                current_x_, current_y_, goal_x_, goal_y_);

            if (planPathAStar(current_x_, current_y_, goal_x_, goal_y_, planned_path_)) {
                path_index_ = 0;
                path_planned_ = true;
            }
            else
            {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(),
                    2000, "A* path planning failed — waiting for valid route.");
                publishStop();
                return;
            }
        }

        if (planned_path_.empty() || path_index_ >= planned_path_.size()) {
            publishStop();
            return;
        }

        const double dx = goal_x_ - current_x_;
        const double dy = goal_y_ - current_y_;
        const double dz = goal_z_ - current_z_;
        const double horizontal_distance = std::sqrt(dx * dx + dy * dy);
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
            1000,
            "NAVIGATE | Position: (%.2f, %.2f, %.2f) | Yaw: %.1f deg | "
            "Distance to goal: %.2f m | Waypoint %zu/%zu",
            current_x_, current_y_, current_z_,
            current_yaw_ * 180.0 / M_PI, distance,
            path_index_ + 1, planned_path_.size());

        // Goal reached check
        const double horizontal_tolerance = 0.5;
        const double altitude_tolerance = 0.3;
        if (horizontal_distance < horizontal_tolerance &&
            std::abs(dz) < altitude_tolerance) {
            RCLCPP_INFO(this->get_logger(), "GOAL REACHED!");
            RCLCPP_INFO(this->get_logger(), "Position: x=%.2f y=%.2f z=%.2f",
                current_x_, current_y_, current_z_);
            publishStop();
            stuck_reversal_count_ = 0;
            state_ = State::LAND;
            return;
        }

        geometry_msgs::msg::Twist cmd;

        // Altitude hold
        const double kp_z = 1.0;
        cmd.linear.z = kp_z * dz;
        cmd.linear.z = std::clamp(cmd.linear.z, -max_z_speed_, max_z_speed_);

        // LiDAR collision safety net
        if (lidar_received_ && front_obstacle_) {
            RCLCPP_INFO(this->get_logger(), "Obstacle detected! Front blocked.");

            int candidate_direction = (left_distance_ > right_distance_) ? 1 : -1;

            const double probe_yaw =
                current_yaw_ + candidate_direction * (M_PI / 2.0);

            const double probe_x =
                current_x_ + std::cos(probe_yaw) * map_probe_distance_;

            const double probe_y =
                current_y_ + std::sin(probe_yaw) * map_probe_distance_;

            if (isPathBlocked(current_x_, current_y_, probe_x, probe_y)) {
                RCLCPP_INFO(this->get_logger(),
                    "Map shows the preferred side is blocked further out — switching avoidance direction.");
                candidate_direction = -candidate_direction;
            }

            avoidance_direction_ = candidate_direction;

            RCLCPP_INFO(this->get_logger(), "Choosing %s avoidance. Left=%.2f Right=%.2f",
                avoidance_direction_ == 1 ? "LEFT" : "RIGHT",
                left_distance_, right_distance_);

            avoid_target_yaw_ = current_yaw_ + avoidance_direction_ * (M_PI / 4.0);
            avoid_target_yaw_ = normalizeAngle(avoid_target_yaw_);

            state_ = State::AVOID_OBSTACLE;
            publishStop();

            RCLCPP_INFO(this->get_logger(), "Starting obstacle avoidance.");
            return;
        }

        // Steer strictly to current A* waypoint
        double target_x = planned_path_[path_index_].first;
        double target_y = planned_path_[path_index_].second;

        double wp_dx = target_x - current_x_;
        double wp_dy = target_y - current_y_;
        double wp_dist = std::sqrt(wp_dx * wp_dx + wp_dy * wp_dy);

        if (wp_dist < waypoint_tolerance_ && path_index_ + 1 < planned_path_.size()) {
            ++path_index_;
            target_x = planned_path_[path_index_].first;
            target_y = planned_path_[path_index_].second;
            wp_dx = target_x - current_x_;
            wp_dy = target_y - current_y_;
            wp_dist = std::sqrt(wp_dx * wp_dx + wp_dy * wp_dy);
        }

        const double desired_yaw = std::atan2(wp_dy, wp_dx);
        double yaw_error = desired_yaw - current_yaw_;
        yaw_error = normalizeAngle(yaw_error);

        const double kp_yaw = 1.2;
        cmd.angular.z = kp_yaw * yaw_error;
        cmd.angular.z = std::clamp(cmd.angular.z, -max_yaw_speed_, max_yaw_speed_);

        const double yaw_tolerance = 10.0 * M_PI / 180.0;
        if (std::abs(yaw_error) < yaw_tolerance) {
            const double kp_forward = 0.25;
            cmd.linear.x = kp_forward * wp_dist;
            cmd.linear.x = std::clamp(cmd.linear.x, 0.0, max_forward_speed_);
        }
        else
        {
            cmd.linear.x = 0.0;
        }

        cmd.linear.y = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;

        cmd_vel_pub_->publish(cmd);
    }

    void avoid_obstacle() {
        geometry_msgs::msg::Twist cmd;

        double yaw_error = avoid_target_yaw_ - current_yaw_;
        yaw_error = normalizeAngle(yaw_error);

        const double dz = goal_z_ - current_z_;

        cmd.linear.z = std::clamp(1.0 * dz, -max_z_speed_, max_z_speed_);
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;

        const double kp_avoid_yaw = 1.0;
        cmd.angular.z = kp_avoid_yaw * yaw_error;
        cmd.angular.z = std::clamp(
            cmd.angular.z, -obstacle_turn_speed_, obstacle_turn_speed_);

        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;

        cmd_vel_pub_->publish(cmd);

        const double yaw_tolerance = 8.0 * M_PI / 180.0;

        if (std::abs(yaw_error) < yaw_tolerance) {
            RCLCPP_INFO(this->get_logger(), "Obstacle avoidance rotation complete.");
            RCLCPP_INFO(this->get_logger(), "Starting to pass obstacle.");

            publishStop();

            pass_start_x_ = current_x_;
            pass_start_y_ = current_y_;

            last_progress_x_ = current_x_;
            last_progress_y_ = current_y_;
            last_progress_time_ = this->now();

            state_ = State::PASS_OBSTACLE;
        }
        else
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Avoiding obstacle | Current yaw=%.1f deg | Target yaw=%.1f deg",
                current_yaw_ * 180.0 / M_PI,
                avoid_target_yaw_ * 180.0 / M_PI);
        }
    }

    void pass_obstacle() {
        geometry_msgs::msg::Twist cmd;

        const double dz = goal_z_ - current_z_;

        cmd.linear.z = std::clamp(1.0 * dz, -max_z_speed_, max_z_speed_);
        cmd.linear.x = obstacle_pass_speed_;
        cmd.linear.y = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;

        double yaw_error = avoid_target_yaw_ - current_yaw_;
        yaw_error = normalizeAngle(yaw_error);

        cmd.angular.z = 0.8 * yaw_error;
        cmd.angular.z = std::clamp(cmd.angular.z, -0.3, 0.3);

        cmd_vel_pub_->publish(cmd);

        const double dist_traveled = std::sqrt(
            std::pow(current_x_ - pass_start_x_, 2) +
            std::pow(current_y_ - pass_start_y_, 2));

        // Replan after reactive detour
        if (!front_obstacle_ && dist_traveled > min_pass_distance_) {
            RCLCPP_INFO(this->get_logger(),
                "Obstacle cleared. Replanning A* route from position (%.2f, %.2f).",
                current_x_, current_y_);

            publishStop();
            stuck_reversal_count_ = 0;
            path_planned_ = false;
            state_ = State::NAVIGATE;
            return;
        }

        const double moved_since_check = std::sqrt(
            std::pow(current_x_ - last_progress_x_, 2) +
            std::pow(current_y_ - last_progress_y_, 2));

        if (moved_since_check > progress_epsilon_) {
            last_progress_x_ = current_x_;
            last_progress_y_ = current_y_;
            last_progress_time_ = this->now();
        }

        const double stalled_for_seconds =
            (this->now() - last_progress_time_).seconds();

        // Chokepoint detection
        if (stalled_for_seconds > stuck_timeout_) {
            const double dist_from_last_reversal = std::sqrt(
                std::pow(current_x_ - last_reversal_x_, 2) +
                std::pow(current_y_ - last_reversal_y_, 2));

            if (stuck_reversal_count_ > 0 && dist_from_last_reversal < chokepoint_distance_) {
                stuck_reversal_count_++;
            }
            else
            {
                stuck_reversal_count_ = 1;
                last_reversal_x_ = current_x_;
                last_reversal_y_ = current_y_;
            }

            if (stuck_reversal_count_ > max_stuck_reversals_) {
                RCLCPP_WARN(this->get_logger(),
                    "Direction reversal hasn't worked %d times at (%.2f, %.2f) — "
                    "chokepoint detected! Handing back to A* planner.",
                    max_stuck_reversals_, current_x_, current_y_);

                stuck_reversal_count_ = 0;
                path_planned_ = false;
                publishStop();
                state_ = State::NAVIGATE;
                return;
            }

            RCLCPP_WARN(this->get_logger(),
                "Stuck while passing obstacle (%.1f s, %.2f m traveled) - "
                "reversing avoidance direction (attempt %d/%d).",
                stalled_for_seconds, dist_traveled,
                stuck_reversal_count_, max_stuck_reversals_);

            avoidance_direction_ = -avoidance_direction_;
            avoid_target_yaw_ = normalizeAngle(
                current_yaw_ + avoidance_direction_ * (M_PI / 2.0));

            publishStop();
            state_ = State::AVOID_OBSTACLE;
            return;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Passing obstacle | Front obstacle=%s | "
            "Distance traveled=%.2f m | Left=%.2f Right=%.2f",
            front_obstacle_ ? "YES" : "NO",
            dist_traveled, left_distance_, right_distance_);
    }

    void land() {
        publishStop();

        std_msgs::msg::Empty land_msg;
        land_pub_->publish(land_msg);

        if (!landing_sent_) {
            RCLCPP_INFO(this->get_logger(), "LAND command sent.");
            landing_sent_ = true;
        }

        if (current_z_ < 0.25) {
            RCLCPP_INFO(this->get_logger(), "Landing complete.");
            state_ = State::DONE;
        }
        else
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Landing... altitude = %.2f m", current_z_);
        }
    }

    double normalizeAngle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    void publishStop() {
        geometry_msgs::msg::Twist cmd;

        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = 0.0;

        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;

        cmd_vel_pub_->publish(cmd);
    }

    // publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr takeoff_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr land_pub_;

    // subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    // timer
    rclcpp::TimerBase::SharedPtr control_timer_;

    // state
    State state_;
    bool odom_received_;
    bool lidar_received_;
    bool front_obstacle_;
    bool takeoff_sent_;
    bool landing_sent_;

    // current drone state
    double current_x_;
    double current_y_;
    double current_z_;
    double current_yaw_;

    // goal
    double goal_x_;
    double goal_y_;
    double goal_z_;

    // lidar
    double left_distance_;
    double right_distance_;
    double front_distance_;
    double obstacle_distance_ = 1.0;
    double front_width_ = 0.6;
    double lidar_max_range_ = 10.0;

    // Static map & planner
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    int8_t occupied_threshold_ = 50;
    std::vector<uint8_t> inflated_occupied_;
    double safety_radius_ = 0.5;
    double map_probe_distance_ = 2.0;
    bool map_frame_logged_ = false;

    std::vector<std::pair<double, double>> planned_path_;
    size_t path_index_;
    bool path_planned_;
    double waypoint_tolerance_ = 0.6;

    // Obstacle avoidance
    int avoidance_direction_;
    double avoid_target_yaw_;
    double pass_start_x_;
    double pass_start_y_;
    double min_pass_distance_ = 0.25;

    double last_progress_x_;
    double last_progress_y_;
    rclcpp::Time last_progress_time_;
    double progress_epsilon_ = 0.15;
    double stuck_timeout_ = 3.0;

    // Chokepoint & loop detection
    int stuck_reversal_count_;
    int max_stuck_reversals_;
    double last_reversal_x_;
    double last_reversal_y_;
    double chokepoint_distance_;

    // control parameters
    double takeoff_height_ = 0.9;
    double max_forward_speed_ = 0.5;
    double obstacle_pass_speed_ = 0.25;
    double max_z_speed_ = 0.4;
    double max_yaw_speed_ = 0.8;
    double obstacle_turn_speed_ = 0.5;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousExplorationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}