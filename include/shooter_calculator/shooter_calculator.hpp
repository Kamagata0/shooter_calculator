#ifndef SHOOTER_CALCULATOR__SHOOTER_CALCULATOR_HPP_
#define SHOOTER_CALCULATOR__SHOOTER_CALCULATOR_HPP_

#include <vector>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>

namespace shooter_calculator
{

// PWM ↔ 初速度 ルックアップテーブル
struct ShooterLUT
{
  int pwm_value;
  double initial_velocity; // m/s
};

class ShooterCalculator
{
public:
  // コンストラクタ
  ShooterCalculator();

  // PWMから初速度を取得（線形補間）
  double getVelocityFromPWM(int pwm_value);

  // 距離から必要なPWMを逆算
  int getPWMFromDistance(double distance);

  // 弾道軌跡を計算（45度固定）
  std::vector<geometry_msgs::msg::Point> calculateTrajectory(
    double initial_velocity,
    int num_points = 50
  );

  // シューターの仕様
  static constexpr double SHOOTER_ANGLE_DEG = 45.0;
  static constexpr double GRAVITY = 9.81;
  static constexpr double SHOOTER_HEIGHT = 0.3;    // ロボット床面からの高さ (m)
  static constexpr double SHOOTER_HORIZONTAL = 0.2; // base_linkからの水平距離 (m)

private:
  std::vector<ShooterLUT> lut_;

  // ルックアップテーブルの初期化（仮データ）
  void initializeLUT();

  // 線形補間
  double linearInterpolate(int pwm, const ShooterLUT & lut1, const ShooterLUT & lut2);
};

} // namespace shooter_calculator

#endif // SHOOTER_CALCULATOR__SHOOTER_CALCULATOR_HPP_
