#include "shooter_calculator/shooter_calculator.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace shooter_calculator
{

class ShooterCalculatorNode : public rclcpp::Node
{
public:
  ShooterCalculatorNode(const rclcpp::NodeOptions & options)
  : Node("shooter_calculator", options),
    calculator_(std::make_unique<ShooterCalculator>())
  {
    // ==========================================
    // パラメータの宣言（ここで各種設定値を定義しています）
    // ==========================================
    
    // ターゲット（バケツ）のTFフレーム名
    this->declare_parameter("target_frame", "moving_bucket");
    this->declare_parameter("target_frame_fallback", "moving_bucket");
    
    // ロボット自身の基準フレーム名
    this->declare_parameter("robot_frame", "base_link");
    
    // 発射角度（度）: 45度固定で計算する場合のデフォルト値
    this->declare_parameter("launch_angle_deg", 45.0);
    
    // 雑巾（弾）の質量 [kg]: シミュレーションの物理計算で使用
    this->declare_parameter("cloth_mass_kg", 0.12);
    
    // 【空気抵抗パラメータ 1】受風面積 [m^2]
    // ※実際の面積より小さく（0.01等に）設定し、空気抵抗による急ブレーキを防いでいます
    this->declare_parameter("cloth_area_m2", 0.01);
    
    // 【空気抵抗パラメータ 2】空気抵抗係数
    // ※数値を小さくすることで、遠くまで届くように調整しています
    this->declare_parameter("drag_coefficient", 0.1);

    // TFバッファとリスナーの初期化
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // RViz用のマーカー配信パブリッシャー
    pub_trajectory_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/trajectory_markers", 10);

    // 0.1秒（100ミリ秒）ごとに計算と可視化を実行するタイマー
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

  geometry_msgs::msg::TransformStamped lookupTargetTransform()
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
      // TFから目標とロボットの位置を取得
      geometry_msgs::msg::TransformStamped transform_mb = lookupTargetTransform();
      const auto robot_frame = this->get_parameter("robot_frame").as_string();
      geometry_msgs::msg::TransformStamped transform_base =
        tf_buffer_->lookupTransform("map", robot_frame, rclcpp::Time());

      double target_x = transform_mb.transform.translation.x;
      double target_y = transform_mb.transform.translation.y;
      double target_z = transform_mb.transform.translation.z;

      double robot_x = transform_base.transform.translation.x;
      double robot_y = transform_base.transform.translation.y;

      // ==========================================
      // 発射口の位置オフセット設定
      // ==========================================
      // ロボットの中心(base_link)から見て、発射口がどこにあるかのオフセット [m]
      const double shooter_x = robot_x + 0.2; // 前方に20cm
      const double shooter_y = robot_y;       // 左右のズレなし
      const double shooter_z = 0.3;           // 床面から高さ30cm
      
      // 距離の計算
      const double dx_to_target = target_x - shooter_x;
      const double dy_to_target = target_y - shooter_y;
      const double horizontal_target_distance = std::sqrt(dx_to_target * dx_to_target + 
                                                          dy_to_target * dy_to_target);
      const double target_height_delta = target_z - shooter_z;

      // パラメータの取得
      const double launch_angle_deg = this->get_parameter("launch_angle_deg").as_double();
      const double cloth_mass = this->get_parameter("cloth_mass_kg").as_double();
      const double cloth_area = this->get_parameter("cloth_area_m2").as_double();
      const double drag_coefficient = this->get_parameter("drag_coefficient").as_double();
      const double angle_rad = launch_angle_deg * M_PI / 180.0;

      // 空気抵抗を考慮した必要初速の算出
      double required_velocity = calculator_->getRequiredVelocityWithDrag(
        horizontal_target_distance, target_height_delta, launch_angle_deg, 
        cloth_mass, cloth_area, drag_coefficient);

      // 初速からPWM値への変換（LUTを使用）
      int required_pwm = calculator_->getPWMFromVelocity(required_velocity);

      // 弾道軌跡のシミュレーション計算
      std::vector<geometry_msgs::msg::Point> target_trajectory;
      
      if (horizontal_target_distance > 0.0) {
        const double ux = dx_to_target / horizontal_target_distance;
        const double uy = dy_to_target / horizontal_target_distance;
        const double air_density = 1.225; // 空気密度 [kg/m^3]
        const double dt = 0.02;           // シミュレーションの刻み時間 [秒]

        double x = shooter_x;
        double y = shooter_y;
        double z = shooter_z;
        double vx = required_velocity * std::cos(angle_rad) * ux;
        double vy = required_velocity * std::cos(angle_rad) * uy;
        double vz = required_velocity * std::sin(angle_rad);

        for (int i = 0; i < 200; ++i) {
          const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
          if (speed < 1e-6) break;

          const double drag_factor = 0.5 * air_density * drag_coefficient * cloth_area / std::max(cloth_mass, 0.05);
          const double ax = -drag_factor * speed * vx;
          const double ay = -drag_factor * speed * vy;
          const double az = -calculator_->GRAVITY - drag_factor * speed * vz;

          vx += ax * dt;
          vy += ay * dt;
          vz += az * dt;
          x += vx * dt;
          y += vy * dt;
          z += vz * dt;

          geometry_msgs::msg::Point p;
          p.x = x; p.y = y; p.z = z;
          
          const double dist_to_target = std::sqrt((x - target_x)*(x - target_x) + 
                                                  (y - target_y)*(y - target_y) + 
                                                  (z - target_z)*(z - target_z));
          if (dist_to_target < 0.15) {
            p.x = target_x; p.y = target_y; p.z = target_z;
            target_trajectory.push_back(p);
            break;
          }
          if (z < 0.0) break;
          target_trajectory.push_back(p);
        }
      }

      // ==========================================
      // RViz マーカー作成（可視化の設定）
      // ==========================================
      visualization_msgs::msg::MarkerArray marker_array;

      // 0: 赤球（目標のバケツ位置）
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
      target_marker.scale.x = 0.1; target_marker.scale.y = 0.1; target_marker.scale.z = 0.1;
      target_marker.color.r = 1.0; target_marker.color.g = 0.0; target_marker.color.b = 0.0; target_marker.color.a = 1.0;
      marker_array.markers.push_back(target_marker);

      // 1: 黄球（ロボットの発射口位置）
      visualization_msgs::msg::Marker shooter_marker;
      shooter_marker.header.frame_id = "map";
      shooter_marker.header.stamp = this->now();
      shooter_marker.ns = "shooter";
      shooter_marker.id = 1;
      shooter_marker.type = visualization_msgs::msg::Marker::SPHERE;
      shooter_marker.action = visualization_msgs::msg::Marker::ADD;
      shooter_marker.pose.position.x = shooter_x;
      shooter_marker.pose.position.y = shooter_y;
      shooter_marker.pose.position.z = shooter_z;
      shooter_marker.scale.x = 0.08; shooter_marker.scale.y = 0.08; shooter_marker.scale.z = 0.08;
      shooter_marker.color.r = 1.0; shooter_marker.color.g = 1.0; shooter_marker.color.b = 0.0; shooter_marker.color.a = 1.0;
      marker_array.markers.push_back(shooter_marker);

      // 2: 緑の線（予測弾道の軌跡）
      visualization_msgs::msg::Marker trajectory_marker;
      trajectory_marker.header.frame_id = "map";
      trajectory_marker.header.stamp = this->now();
      trajectory_marker.ns = "shooter";
      trajectory_marker.id = 2;
      trajectory_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      trajectory_marker.action = visualization_msgs::msg::Marker::ADD;
      trajectory_marker.scale.x = 0.02; // 線の太さ
      trajectory_marker.color.r = 0.0; trajectory_marker.color.g = 1.0; trajectory_marker.color.b = 0.0; trajectory_marker.color.a = 0.8;
      for (const auto & p : target_trajectory) {
        trajectory_marker.points.push_back(p);
      }
      marker_array.markers.push_back(trajectory_marker);

      // 3: 青球（着地予測点）
      if (!target_trajectory.empty()) {
        visualization_msgs::msg::Marker landing_marker;
        landing_marker.header.frame_id = "map";
        landing_marker.header.stamp = this->now();
        landing_marker.ns = "shooter";
        landing_marker.id = 3;
        landing_marker.type = visualization_msgs::msg::Marker::SPHERE;
        landing_marker.action = visualization_msgs::msg::Marker::ADD;
        landing_marker.pose.position.x = target_x;
        landing_marker.pose.position.y = target_y;
        landing_marker.pose.position.z = target_z;
        landing_marker.scale.x = 0.15; landing_marker.scale.y = 0.15; landing_marker.scale.z = 0.15;
        landing_marker.color.r = 0.0; landing_marker.color.g = 0.0; landing_marker.color.b = 1.0; landing_marker.color.a = 0.6;
        marker_array.markers.push_back(landing_marker);
      }

      // 4: 文字情報（距離・PWM・初速の表示）
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
      info_marker.color.r = 1.0; info_marker.color.g = 1.0; info_marker.color.b = 1.0; info_marker.color.a = 1.0;
      info_marker.text = "Distance: " + std::to_string(horizontal_target_distance) + " m\n" +
                         "PWM: " + std::to_string(required_pwm) + "\n" +
                         "Velocity: " + std::to_string(required_velocity) + " m/s";
      marker_array.markers.push_back(info_marker);

      pub_trajectory_->publish(marker_array);

      // ==========================================
      // ターミナルへのログ出力（1秒に1回）
      // ==========================================
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000, // 1000ms（1秒）間隔
        "Target: %.2f m, Height: %.2f m | Required Vel: %.2f m/s -> [ PWM: %d ]",
        horizontal_target_distance, 
        target_height_delta, 
        required_velocity, 
        required_pwm
      );

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