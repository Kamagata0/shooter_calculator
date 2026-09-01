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
    {1, 1.55, 0.0},
    {2, 2.296666667, 0.533333333},
    {3, 4.763333333, 2.666666667},
    {4, 5.953333333, 3.966666667},
    {5, 8.84, 3.566666667},
    {6, 11.23333333, 4.1},
    {7, 8.516666667, 3.333333333},
    {8, 9.04, 3.566666667},
    {9, 9.42, 3.833333333},
    {10, 10.84666667, 3.966666667}
  };
}

double ShooterCalculator::getReleaseVelocityFromOutputLevel(int output_level)
{
  output_level = std::clamp(output_level, 1, 10);
  for (size_t i = 0; i < lut_.size() - 1; ++i) {
    if (lut_[i].output_level <= output_level && output_level <= lut_[i + 1].output_level) {
      return linearInterpolate(output_level, lut_[i], lut_[i + 1]);
    }
  }
  return lut_.back().release_velocity;
}

int ShooterCalculator::getOutputLevelFromDistance(double distance)
{
  if (distance <= 0.0) {
    return 1;
  }

  for (size_t i = 0; i < lut_.size() - 1; ++i) {
    if (lut_[i].flight_distance <= distance && distance <= lut_[i + 1].flight_distance) {
      double ratio = (distance - lut_[i].flight_distance) /
                     (lut_[i + 1].flight_distance - lut_[i].flight_distance);
      int level = static_cast<int>(lut_[i].output_level +
                                  ratio * (lut_[i + 1].output_level - lut_[i].output_level));
      return std::clamp(level, 1, 10);
    }
  }
  return 10;
}

int ShooterCalculator::getOutputLevelFromVelocity(double required_velocity)
{
  for (size_t i = 0; i < lut_.size() - 1; ++i) {
    if (lut_[i].release_velocity <= required_velocity &&
        required_velocity <= lut_[i + 1].release_velocity)
    {
      double ratio = (required_velocity - lut_[i].release_velocity) /
                     (lut_[i + 1].release_velocity - lut_[i].release_velocity);
      int level = static_cast<int>(lut_[i].output_level +
                                  ratio * (lut_[i + 1].output_level - lut_[i].output_level));
      return std::clamp(level, 1, 10);
    }
  }
  if (required_velocity < lut_.front().release_velocity) return 1;
  return 10;
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
  int output_level,
  const ShooterLUT & lut1,
  const ShooterLUT & lut2)
{
  double ratio = static_cast<double>(output_level - lut1.output_level) /
                 (lut2.output_level - lut1.output_level);
  return lut1.release_velocity + ratio * (lut2.release_velocity - lut1.release_velocity);
}

double ShooterCalculator::linearInterpolateDistance(
  double distance,
  const ShooterLUT & lut1,
  const ShooterLUT & lut2)
{
  double ratio = (distance - lut1.flight_distance) /
                 (lut2.flight_distance - lut1.flight_distance);
  return lut1.release_velocity + ratio * (lut2.release_velocity - lut1.release_velocity);
}

} // namespace shooter_calculator