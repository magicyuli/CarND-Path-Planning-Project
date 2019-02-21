#include "path_planner.h"

#include <iostream>

#include "math.h"
#include "spline.h"
#include "utils.h"

using std::pair;
using std::unordered_map;
using std::vector;

// Max speed m/sec
static double MAX_SPEED_MPS = 22.0;
// Max acceleration m/sec^2
static double MAX_ACC_MPS2 = 5.0;
// Simulator step interval sec
static double INTERVAL_S = 0.02;
// Safe distance to other cars for changing lanes and following
static double SAFE_DIST_S = 20.0;

namespace car {

pair<vector<double>, vector<double>> PathPlanner::Plan(const Input& d) {
  auto cars_front_back = GetCarsInFrontAndBehind(d.sensor_data, d.car_s);
  
  double target_v_ms = CalculateTargetSpeed(cars_front_back);

  Lane target_lane = CalculateTargetLane(cars_front_back);

  std::cout << "\n\ncar_v: " << v_ms_ << "; target_v_ms: " << target_v_ms << "; target_lane: " << target_lane << std::endl;

  // Change lane
  lane_ = target_lane;

  vector<double> spline_ref_x;
  vector<double> spline_ref_y;
  bool has_prev_path = d.prev_path_x.size() > 1;
  // Make sure the spline fit passes points closest to our car on the previous path, or our car
  // if there's no previous path
  if (has_prev_path) {
    spline_ref_x.push_back(d.prev_path_x[0]);
    spline_ref_x.push_back(d.prev_path_x[1]);
    spline_ref_y.push_back(d.prev_path_y[0]);
    spline_ref_y.push_back(d.prev_path_y[1]);
  }
  else {
    spline_ref_x.push_back(d.car_x);
    spline_ref_y.push_back(d.car_y);
  }
  // Use points far away as reference points to fit spline to make paths smooth
  for (int i = 30; i <= 90; i += 30) {
    auto xy = utils::GetXY(d.car_s + i, lane_ * 4 + 2, map_waypoints_s_, map_waypoints_x_, map_waypoints_y_);
    spline_ref_x.push_back(xy[0]);
    spline_ref_y.push_back(xy[1]);
  }
  // Convert reference points to car coordinates
  auto car_spline_ref_xy = utils::MapXYToCarXY(spline_ref_x, spline_ref_y, d.car_x, d.car_y, d.car_yaw_rad);

  // spline fit
  tk::spline s;
  s.set_points(car_spline_ref_xy.first, car_spline_ref_xy.second);

  // Result vectors
  vector<double> car_path_x;
  vector<double> car_path_y;
  // x coordinate used to generate path points from spline
  double x;
  if (has_prev_path) {
    car_path_x.push_back(car_spline_ref_xy.first[0]);
    car_path_y.push_back(car_spline_ref_xy.second[0]);
    car_path_x.push_back(car_spline_ref_xy.first[1]);
    car_path_y.push_back(car_spline_ref_xy.second[1]);
    x = car_spline_ref_xy.first[1];
  }
  else {
    x = car_spline_ref_xy.first[0];
  }

  double a = CalculateAcceleration(v_ms_, target_v_ms);
  v_ms_ += a * INTERVAL_S;
  for (int i = 0; i < 50; i++) {
    x += v_ms_ * INTERVAL_S;
    car_path_x.push_back(x);
    car_path_y.push_back(s(x));
  }

  // Convert points from car coordinates back to map cooridinates
  return utils::CarXYToMapXY(car_path_x, car_path_y, d.car_x, d.car_y, d.car_yaw_rad);
}

// Return cars in front and behind within the safe distance on all lanes
unordered_map<Lane, vector<Car>> PathPlanner::GetCarsInFrontAndBehind(vector<Car> cars, double my_s) {
  unordered_map<Lane, vector<Car>> ret{
    make_pair(0, vector<Car>{Car(), Car()}),
    make_pair(1, vector<Car>{Car(), Car()}),
    make_pair(2, vector<Car>{Car(), Car()})
  };
  for (auto car : cars) {
    double car_s = car[5];
    double car_d = car[6];
    Lane car_lane = (int) car_d / 4;
    if (fabs(car_s - my_s) > SAFE_DIST_S || car_lane < 0 || car_lane > 2) {
      continue;
    }

    if (car_s > my_s) {
      auto cur_in_front = ret[car_lane][0];
      std::cout << "cur_in_front (lane " << car_lane << "): ";
      utils::PrintVec(car);
      if (cur_in_front.empty() || cur_in_front[5] > car_s) {
        ret[car_lane][0]= car;
      }
    }
    else {
      auto cur_behind = ret[car_lane][1];
      std::cout << "cur_behind (lane " << car_lane << "): ";
      utils::PrintVec(car);
      if (cur_behind.empty() || cur_behind[5] < car_s) {
        ret[car_lane][1] = car;
      }
    }
  }
  return ret;
}

// Return max speed if there's no car is in front within the safe distance, or
// the speed of the car in front
double PathPlanner::CalculateTargetSpeed(unordered_map<Lane, vector<Car>> cars_front_back) {
  Car car_front_same_lane = cars_front_back[lane_][0];
  if (car_front_same_lane.empty()) {
    return MAX_SPEED_MPS;
  }
  double v_x = car_front_same_lane[3];
  double v_y = car_front_same_lane[4];
  double v = sqrt(v_x * v_x + v_y * v_y);
  return utils::MphToMps(v);
}

// Return the current lane if there's no car in front within the safe distance, or
// an adjacent lane if there's no cars in front or behind within the safe distance
Lane PathPlanner::CalculateTargetLane(std::unordered_map<Lane, std::vector<Car>> cars_front_back) {
  Car car_front_same_lane = cars_front_back[lane_][0];
  if (car_front_same_lane.empty()) {
    return lane_;
  }
  Lane left_lane = lane_ - 1;
  Lane right_lane = lane_ + 1;
  if (left_lane >= 0) {
    auto cars_left = cars_front_back[left_lane];
    if (cars_left[1].empty() && cars_left[0].empty()) {
      return left_lane;
    }
  }
  if (right_lane <= 2) {
    auto cars_right = cars_front_back[right_lane];
    if (cars_right[1].empty() && cars_right[0].empty()) {
      return right_lane;
    }
  }
  return lane_;
}

// Return 0 if the current speed is close enough to the target speed, or
// +/-MAX_ACC_MPS2 depending on whether we need to accelerate or decelerate
double PathPlanner::CalculateAcceleration(double cur_v_mps, double target_v_mps) {
  double diff = target_v_mps - cur_v_mps;
  if (fabs(diff) / INTERVAL_S < MAX_ACC_MPS2) {
    return 0;
  }
  return diff / fabs(diff) * MAX_ACC_MPS2;
}

} // namespace car