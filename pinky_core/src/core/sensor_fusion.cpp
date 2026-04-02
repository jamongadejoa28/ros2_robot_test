#include "pinky_core/core/sensor_fusion.h"
#include <cmath>

namespace pinky {

SensorFusion::SensorFusion(double initial_x, double initial_y, double initial_theta)
    : x_(initial_x), y_(initial_y), theta_(initial_theta) {}

bool SensorFusion::Predict(double v, double w, Timestamp now) {
  if (!initialized_) {
    last_predict_time_ = now;
    initialized_ = true;
    return false;
  }

  double dt_ns = static_cast<double>(now.nanoseconds - last_predict_time_.nanoseconds);
  double dt = dt_ns / 1e9;
  if (dt <= 0.0) return false;

  // Fuse angular velocity
  // If IMU data is recent (e.g. within 100ms), apply complementary filter.
  // Otherwise, fallback to pure wheel odometry.
  double current_w = w;
  if (has_imu_) {
    double imu_dt_ns = static_cast<double>(now.nanoseconds - last_imu_time_.nanoseconds);
    if (imu_dt_ns / 1e9 < 0.1) { 
      current_w = alpha_ * last_imu_yaw_rate_ + (1.0 - alpha_) * w;
    } else {
      has_imu_ = false; // IMU data is stale
    }
  }

  // Kinematic integration
  double delta_distance = v * dt;
  double delta_theta = current_w * dt;

  theta_ += delta_theta;
  
  // Normalize theta to [-pi, pi]
  while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
  while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

  x_ += delta_distance * std::cos(theta_);
  y_ += delta_distance * std::sin(theta_);

  vx_ = v;
  vth_ = current_w;

  last_predict_time_ = now;
  return true;
}

void SensorFusion::UpdateImu(double imu_yaw_rate, Timestamp now) {
  last_imu_yaw_rate_ = imu_yaw_rate;
  last_imu_time_ = now;
  has_imu_ = true;
}

void SensorFusion::Reset(double x, double y, double theta) {
  x_ = x;
  y_ = y;
  theta_ = theta;
  vx_ = 0.0;
  vth_ = 0.0;
  initialized_ = false;
  has_imu_ = false;
}

Odometry SensorFusion::GetState(Timestamp stamp) const {
  return {stamp, x_, y_, theta_, vx_, vth_};
}

} // namespace pinky