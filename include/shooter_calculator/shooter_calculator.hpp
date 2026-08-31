#ifndef SHOOTER_CALCULATOR__SHOOTER_CALCULATOR_HPP_
#define SHOOTER_CALCULATOR__SHOOTER_CALCULATOR_HPP_

#include <vector>
#include <geometry_msgs/msg/point.hpp>

namespace shooter_calculator
{

struct ShooterLUT {
  int pwm_value;
  double initial_velocity;
};

class ShooterCalculator
{
public:
  ShooterCalculator();

  void initializeLUT();
  double getVelocityFromPWM(int pwm_value);
  
  // 新しく追加した関数群
  int getPWMFromVelocity(double required_velocity);
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
  const double SHOOTER_ANGLE_DEG = 45.0;
  const double SHOOTER_HORIZONTAL = 0.2;
  const double SHOOTER_HEIGHT = 0.3;

private:
  std::vector<ShooterLUT> lut_;
  double linearInterpolate(int pwm, const ShooterLUT & lut1, const ShooterLUT & lut2);
};

}  // namespace shooter_calculator

#endif  // SHOOTER_CALCULATOR__SHOOTER_CALCULATOR_HPP_