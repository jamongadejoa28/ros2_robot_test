#pragma once

#include "pinky_core/common/types.h"

namespace pinky {

// A lightweight Complementary Filter for 2D Sensor Fusion
// Fuses raw wheel odometry (linear velocity) with IMU (yaw rate)
// to provide a more accurate, slip-resistant 2D pose estimate.
class SensorFusion {
 public:
  SensorFusion(double initial_x = 0.0, double initial_y = 0.0, double initial_theta = 0.0);

  // Update step with wheel odometry increments (v, w from encoders)
  // Returns true if state was updated.
  bool Predict(double v, double w, Timestamp now);

  // Update step with IMU angular velocity (yaw rate)
  void UpdateImu(double imu_yaw_rate, Timestamp now);

  void Reset(double x, double y, double theta);

  Odometry GetState(Timestamp stamp) const;

 private:
  double x_{0.0};
  double y_{0.0};
  double theta_{0.0};
  
  double vx_{0.0};
  double vth_{0.0};
  
  double last_imu_yaw_rate_{0.0};
  bool has_imu_{false};
  Timestamp last_imu_time_{0};
  
  Timestamp last_predict_time_{0};
  bool initialized_{false};

  // Complementary filter weight for IMU yaw rate vs Odom yaw rate.
  // 1.0 = Trust IMU 100%, 0.0 = Trust Odom 100%
  double alpha_{0.9}; 
};

} // namespace pinky