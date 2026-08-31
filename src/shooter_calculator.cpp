#include "shooter_calculator/shooter_calculator.hpp"
#include <algorithm>
#include <cmath>

namespace shooter_calculator
{

ShooterCalculator::ShooterCalculator()
{
  initializeLUT();
}

void ShooterCalculator::initializeLUT()
{
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
  pwm_value = std::clamp(pwm_value, 0, 255);
  for (size_t i = 0; i < lut_.size() - 1; ++i) {
    if (lut_[i].pwm_value <= pwm_value && pwm_value <= lut_[i + 1].pwm_value) {
      return linearInterpolate(pwm_value, lut_[i], lut_[i + 1]);
    }
  }
  return lut_.back().initial_velocity;
}

int ShooterCalculator::getPWMFromVelocity(double required_velocity)
{
  for (size_t i = 0; i < lut_.size() - 1; ++i) {
    if (lut_[i].initial_velocity <= required_velocity &&
        required_velocity <= lut_[i + 1].initial_velocity)
    {
      double ratio = (required_velocity - lut_[i].initial_velocity) /
                     (lut_[i + 1].initial_velocity - lut_[i].initial_velocity);
      int pwm = static_cast<int>(lut_[i].pwm_value +
                                 ratio * (lut_[i + 1].pwm_value - lut_[i].pwm_value));
      return std::clamp(pwm, 0, 255);
    }
  }
  if (required_velocity < lut_.front().initial_velocity) return 0;
  return 255;
}

double ShooterCalculator::getRequiredVelocityWithDrag(
  double horizontal_distance,
  double height_diff,
  double angle_deg,
  double mass,
  double area,
  double drag_coeff)
{
  double min_vel = 1.0;
  double max_vel = 20.0;
  double best_vel = 5.0;

  const double angle_rad = angle_deg * M_PI / 180.0;
  const double air_density = 1.225;
  const double drag_factor = 0.5 * air_density * drag_coeff * area / std::max(mass, 0.01);
  const double dt = 0.005;

  for (int iter = 0; iter < 30; ++iter) {
    best_vel = (min_vel + max_vel) / 2.0;

    double x = 0.0;
    double z = 0.0;
    double vx = best_vel * std::cos(angle_rad);
    double vz = best_vel * std::sin(angle_rad);

    bool reached_distance = false;
    double final_z = -999.0;

    for (int i = 0; i < 1000; ++i) {
      double speed = std::sqrt(vx * vx + vz * vz);
      double ax = -drag_factor * speed * vx;
      double az = -GRAVITY - drag_factor * speed * vz;

      vx += ax * dt;
      vz += az * dt;
      x += vx * dt;
      z += vz * dt;

      if (x >= horizontal_distance) {
        final_z = z;
        reached_distance = true;
        break;
      }
      if (z < height_diff - 2.0) {
        break;
      }
    }

    if (reached_distance) {
      if (final_z >= height_diff) {
        max_vel = best_vel;
      } else {
        min_vel = best_vel;
      }
    } else {
      min_vel = best_vel;
    }
  }
  return best_vel;
}

std::vector<geometry_msgs::msg::Point> ShooterCalculator::calculateTrajectory(
  double initial_velocity,
  int num_points)
{
  std::vector<geometry_msgs::msg::Point> trajectory;
  double angle_rad = SHOOTER_ANGLE_DEG * M_PI / 180.0;
  double v_x = initial_velocity * std::cos(angle_rad);
  double v_z = initial_velocity * std::sin(angle_rad);
  double max_distance = (initial_velocity * initial_velocity * std::sin(2 * angle_rad)) / GRAVITY;

  for (int i = 0; i < num_points; ++i) {
    double t = (i * max_distance) / (v_x * (num_points - 1));
    geometry_msgs::msg::Point p;
    p.x = SHOOTER_HORIZONTAL + v_x * t;
    p.y = 0.0;
    p.z = SHOOTER_HEIGHT + v_z * t - 0.5 * GRAVITY * t * t;
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