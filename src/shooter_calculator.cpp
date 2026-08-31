#include "shooter_calculator/shooter_calculator.hpp"

#include <algorithm>

namespace shooter_calculator
{

ShooterCalculator::ShooterCalculator()
{
  initializeLUT();
}

void ShooterCalculator::initializeLUT()
{
  // 仮LUT: PWM値と初速度の関係
  // 実測データを後で置き換えてください
  lut_ = {
    {0, 0.0},       // PWM 0
    {50, 3.5},      // PWM 50
    {100, 6.2},     // PWM 100
    {150, 9.1},     // PWM 150
    {200, 11.8},    // PWM 200
    {255, 14.5},    // PWM 255 (最大)
  };
}

double ShooterCalculator::getVelocityFromPWM(int pwm_value)
{
  // PWM値を0-255に制限
  pwm_value = std::clamp(pwm_value, 0, 255);

  // LUT内で対応する値を探す
  for (size_t i = 0; i < lut_.size() - 1; ++i) {
    if (lut_[i].pwm_value <= pwm_value && pwm_value <= lut_[i + 1].pwm_value) {
      return linearInterpolate(pwm_value, lut_[i], lut_[i + 1]);
    }
  }

  // 最大値を超えた場合
  return lut_.back().initial_velocity;
}

int ShooterCalculator::getPWMFromDistance(double distance)
{
  // 45度で発射した場合の初速度: v₀² = distance * g
  // （最高到達距離 = v₀² / g）
  double required_velocity = std::sqrt(distance * GRAVITY);

  // LUT内で対応するPWM値を探す（逆引き）
  for (size_t i = 0; i < lut_.size() - 1; ++i) {
    if (lut_[i].initial_velocity <= required_velocity &&
        required_velocity <= lut_[i + 1].initial_velocity)
    {
      // 線形補間で逆算
      double ratio = (required_velocity - lut_[i].initial_velocity) /
                     (lut_[i + 1].initial_velocity - lut_[i].initial_velocity);
      int pwm = static_cast<int>(lut_[i].pwm_value +
                                 ratio * (lut_[i + 1].pwm_value - lut_[i].pwm_value));
      return std::clamp(pwm, 0, 255);
    }
  }

  // 必要な速度が最大値を超える場合
  return 255;
}

std::vector<geometry_msgs::msg::Point> ShooterCalculator::calculateTrajectory(
  double initial_velocity,
  int num_points)
{
  std::vector<geometry_msgs::msg::Point> trajectory;

  double angle_rad = SHOOTER_ANGLE_DEG * M_PI / 180.0;
  double v_x = initial_velocity * std::cos(angle_rad);
  double v_z = initial_velocity * std::sin(angle_rad);

  // 最大到達距離を計算
  double max_distance = (initial_velocity * initial_velocity * std::sin(2 * angle_rad)) / GRAVITY;
  double dt = max_distance / (num_points - 1);

  for (int i = 0; i < num_points; ++i) {
    double t = (i * max_distance) / (v_x * (num_points - 1));

    geometry_msgs::msg::Point p;
    p.x = SHOOTER_HORIZONTAL + v_x * t;
    p.y = 0.0; // 45度固定で左右方向なし
    p.z = SHOOTER_HEIGHT + v_z * t - 0.5 * GRAVITY * t * t;

    // 地面より下には行かない
    if (p.z >= 0.0) {
      trajectory.push_back(p);
    } else {
      break;
    }
  }

  return trajectory;
}

double ShooterCalculator::linearInterpolate(
  int pwm,
  const ShooterLUT & lut1,
  const ShooterLUT & lut2)
{
  double ratio = static_cast<double>(pwm - lut1.pwm_value) /
                 (lut2.pwm_value - lut1.pwm_value);
  return lut1.initial_velocity + ratio * (lut2.initial_velocity - lut1.initial_velocity);
}

} // namespace shooter_calculator
