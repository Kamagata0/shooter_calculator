#ifndef SHOOTER_CALCULATOR__SHOOTER_CALCULATOR_HPP_
#define SHOOTER_CALCULATOR__SHOOTER_CALCULATOR_HPP_

#include <vector>
#include <geometry_msgs/msg/point.hpp>

namespace shooter_calculator
{

struct ShooterLUT {
  int output_level;
  double release_velocity;
  double flight_distance;
};

class ShooterCalculator
{
public:
  ShooterCalculator();

  void initializeLUT();

  // 実測データベースに基づく出力レベル/リリース速度の計算
  double getReleaseVelocityFromOutputLevel(int output_level);
  int getOutputLevelFromDistance(double distance);
  int getOutputLevelFromVelocity(double required_velocity);

  // 互換ラッパー
  double getVelocityFromPWM(int pwm_value) { return getReleaseVelocityFromOutputLevel(pwm_value); }
  int getPWMFromDistance(double distance) { return getOutputLevelFromDistance(distance); }
  int getPWMFromVelocity(double required_velocity) { return getOutputLevelFromVelocity(required_velocity); }

  double getRequiredVelocityWithDrag(
    double horizontal_distance,
    double height_diff,
    double angle_deg,
    double mass,
    double area,
    double drag_coeff);

  std::vector<geometry_msgs::msg::Point> calculateTrajectory(
    double initial_velocity,
    int num_points = 50);

  const double GRAVITY = 9.81;
  const double SHOOTER_ANGLE_DEG = 50.0;
  const double SHOOTER_HORIZONTAL = 0.2;
  const double SHOOTER_HEIGHT = 0.3;

private:
  std::vector<ShooterLUT> lut_;
  double linearInterpolate(int output_level, const ShooterLUT & lut1, const ShooterLUT & lut2);
  double linearInterpolateDistance(double distance, const ShooterLUT & lut1, const ShooterLUT & lut2);
};

}  // namespace shooter_calculator

#endif  // SHOOTER_CALCULATOR__SHOOTER_CALCULATOR_HPP_