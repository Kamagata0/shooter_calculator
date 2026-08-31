#include "shooter_calculator/shooter_calculator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace shooter_calculator
{

class ShooterCalculatorNode : public rclcpp::Node
{
public:
  ShooterCalculatorNode(const rclcpp::NodeOptions & options)
  : Node("shooter_calculator", options),
    calculator_(std::make_unique<ShooterCalculator>())
  {
    this->declare_parameter("target_frame", "moving_bucket");
    this->declare_parameter("target_frame_fallback", "movingbacket");
    this->declare_parameter("robot_frame", "base_link");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pub_trajectory_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/trajectory_markers", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&ShooterCalculatorNode::calculateAndVisualize, this));

    RCLCPP_INFO(this->get_logger(), "ShooterCalculator initialized");
  }

private:
  std::unique_ptr<ShooterCalculator> calculator_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_trajectory_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::TransformStamped lookupTargetTransform() const
  {
    const auto target_frame = this->get_parameter("target_frame").as_string();
    const auto fallback_target_frame = this->get_parameter("target_frame_fallback").as_string();

    try {
      return tf_buffer_->lookupTransform("map", target_frame, rclcpp::Time());
    } catch (const tf2::TransformException &) {
      if (!fallback_target_frame.empty() && fallback_target_frame != target_frame) {
        try {
          return tf_buffer_->lookupTransform("map", fallback_target_frame, rclcpp::Time());
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Could not resolve target frame '%s' or '%s': %s",
            target_frame.c_str(), fallback_target_frame.c_str(), ex.what());
          throw;
        }
      }
      throw;
    }
  }

  void calculateAndVisualize()
  {
    try {
      // moving target の位置を取得
      geometry_msgs::msg::TransformStamped transform_mb = lookupTargetTransform();

      // ロボットの位置を取得
      const auto robot_frame = this->get_parameter("robot_frame").as_string();
      geometry_msgs::msg::TransformStamped transform_base =
        tf_buffer_->lookupTransform("map", robot_frame, rclcpp::Time());

      double target_x = transform_mb.transform.translation.x;
      double target_y = transform_mb.transform.translation.y;
      double target_z = transform_mb.transform.translation.z;

      double robot_x = transform_base.transform.translation.x;
      double robot_y = transform_base.transform.translation.y;

      // ロボットから目標までの距離（水平）
      double dx = target_x - (robot_x + 0.2); // シューターは0.2m前方
      double dy = target_y - robot_y;
      double horizontal_distance = std::sqrt(dx * dx + dy * dy);

      // 必要なPWM値を計算
      int required_pwm = calculator_->getPWMFromDistance(horizontal_distance);
      double required_velocity = calculator_->getVelocityFromPWM(required_pwm);

      // 弾道軌跡を計算
      auto trajectory = calculator_->calculateTrajectory(required_velocity, 50);

      // マーカーArrayを作成
      visualization_msgs::msg::MarkerArray marker_array;

      // 0: ターゲット位置（赤球）
      visualization_msgs::msg::Marker target_marker;
      target_marker.header.frame_id = "map";
      target_marker.header.stamp = this->now();
      target_marker.ns = "shooter";
      target_marker.id = 0;
      target_marker.type = visualization_msgs::msg::Marker::SPHERE;
      target_marker.action = visualization_msgs::msg::Marker::ADD;
      target_marker.pose.position.x = target_x;
      target_marker.pose.position.y = target_y;
      target_marker.pose.position.z = target_z;
      target_marker.scale.x = 0.1;
      target_marker.scale.y = 0.1;
      target_marker.scale.z = 0.1;
      target_marker.color.r = 1.0;
      target_marker.color.g = 0.0;
      target_marker.color.b = 0.0;
      target_marker.color.a = 1.0;
      marker_array.markers.push_back(target_marker);

      // 1: シューター位置（黄球）
      visualization_msgs::msg::Marker shooter_marker;
      shooter_marker.header.frame_id = "map";
      shooter_marker.header.stamp = this->now();
      shooter_marker.ns = "shooter";
      shooter_marker.id = 1;
      shooter_marker.type = visualization_msgs::msg::Marker::SPHERE;
      shooter_marker.action = visualization_msgs::msg::Marker::ADD;
      shooter_marker.pose.position.x = robot_x + 0.2;
      shooter_marker.pose.position.y = robot_y;
      shooter_marker.pose.position.z = 0.3;
      shooter_marker.scale.x = 0.08;
      shooter_marker.scale.y = 0.08;
      shooter_marker.scale.z = 0.08;
      shooter_marker.color.r = 1.0;
      shooter_marker.color.g = 1.0;
      shooter_marker.color.b = 0.0;
      shooter_marker.color.a = 1.0;
      marker_array.markers.push_back(shooter_marker);

      // 2: 弾道軌跡（緑の線）
      visualization_msgs::msg::Marker trajectory_marker;
      trajectory_marker.header.frame_id = "map";
      trajectory_marker.header.stamp = this->now();
      trajectory_marker.ns = "shooter";
      trajectory_marker.id = 2;
      trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
      trajectory_marker.scale.x = 0.02; // 線の太さ
      trajectory_marker.color.r = 0.0;
      trajectory_marker.color.g = 1.0;
      trajectory_marker.color.b = 0.0;
      trajectory_marker.color.a = 0.8;

      // シューター位置から開始
      geometry_msgs::msg::Point start;
      start.x = robot_x + 0.2;
      start.y = robot_y;
      start.z = 0.3;
      trajectory_marker.points.push_back(start);

      // 弾道軌跡を追加（ロボット座標系からmap座標系に変換）
      for (const auto & p : trajectory) {
        geometry_msgs::msg::Point map_point;
        map_point.x = robot_x + 0.2 + p.x;
        map_point.y = robot_y + p.y;
        map_point.z = p.z;
        trajectory_marker.points.push_back(map_point);
      }

      marker_array.markers.push_back(trajectory_marker);

      // 3: 着弾予測点（大きな円）
      if (!trajectory.empty()) {
        visualization_msgs::msg::Marker landing_marker;
        landing_marker.header.frame_id = "map";
        landing_marker.header.stamp = this->now();
        landing_marker.ns = "shooter";
        landing_marker.id = 3;
        landing_marker.type = visualization_msgs::msg::Marker::SPHERE;
        landing_marker.action = visualization_msgs::msg::Marker::ADD;
        landing_marker.pose.position.x = robot_x + 0.2 + trajectory.back().x;
        landing_marker.pose.position.y = robot_y + trajectory.back().y;
        landing_marker.pose.position.z = trajectory.back().z;
        landing_marker.scale.x = 0.15;
        landing_marker.scale.y = 0.15;
        landing_marker.scale.z = 0.15;
        landing_marker.color.r = 0.0;
        landing_marker.color.g = 0.0;
        landing_marker.color.b = 1.0;
        landing_marker.color.a = 0.6;
        marker_array.markers.push_back(landing_marker);
      }

      // 4: 計算情報（テキスト）
      visualization_msgs::msg::Marker info_marker;
      info_marker.header.frame_id = "map";
      info_marker.header.stamp = this->now();
      info_marker.ns = "shooter";
      info_marker.id = 4;
      info_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      info_marker.action = visualization_msgs::msg::Marker::ADD;
      info_marker.pose.position.x = robot_x + 1.0;
      info_marker.pose.position.y = robot_y;
      info_marker.pose.position.z = 1.5;
      info_marker.scale.z = 0.1;
      info_marker.color.r = 1.0;
      info_marker.color.g = 1.0;
      info_marker.color.b = 1.0;
      info_marker.color.a = 1.0;
      info_marker.text = "Distance: " + std::to_string(horizontal_distance) + " m\n" +
                         "PWM: " + std::to_string(required_pwm) + "\n" +
                         "Velocity: " + std::to_string(required_velocity) + " m/s";
      marker_array.markers.push_back(info_marker);

      pub_trajectory_->publish(marker_array);

      RCLCPP_DEBUG(this->get_logger(),
        "Distance: %.2f m | PWM: %d | Velocity: %.2f m/s",
        horizontal_distance, required_pwm, required_velocity);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Transform error: %s", ex.what());
    }
  }
};

} // namespace shooter_calculator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<shooter_calculator::ShooterCalculatorNode>(
    rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
