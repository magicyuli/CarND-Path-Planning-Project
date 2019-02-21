#ifndef CAR_PATH_PLANNER_H
#define CAR_PATH_PLANNER_H

#include <unordered_map>
#include <vector>

typedef std::vector<double> Car;
typedef int Lane;

namespace car {

struct Input {
  double car_x;
  double car_y;
  double car_yaw_rad;
  double car_s;
  double car_d;
  double car_speed_kph;
  std::vector<double> prev_path_x;
  std::vector<double> prev_path_y;
  std::vector<Car> sensor_data;
};

class PathPlanner
{
public:
  PathPlanner(std::vector<double> map_waypoints_x, std::vector<double> map_waypoints_y, std::vector<double> map_waypoints_s,
   std::vector<double> map_waypoints_dx, std::vector<double> map_waypoints_dy):
   map_waypoints_x_(map_waypoints_x),
   map_waypoints_y_(map_waypoints_y),
   map_waypoints_s_(map_waypoints_s),
   map_waypoints_dx_(map_waypoints_dx),
   map_waypoints_dy_(map_waypoints_dy),
   lane_(1),
   v_ms_(0.0)
  {};

  void Reset(Lane lane, double car_v_ms) {
    lane_ = lane;
    v_ms_ = car_v_ms;
  }

  std::pair<std::vector<double>, std::vector<double>> Plan(const Input& data);
private:
  std::vector<double> map_waypoints_x_;
  std::vector<double> map_waypoints_y_;
  std::vector<double> map_waypoints_s_;
  std::vector<double> map_waypoints_dx_;
  std::vector<double> map_waypoints_dy_;
  Lane lane_;
  double v_ms_;

  std::unordered_map<Lane, std::vector<Car>> GetCarsInFrontAndBehind(std::vector<Car> cars, double my_s);

  double CalculateTargetSpeed(std::unordered_map<Lane, std::vector<Car>> cars_front_back);

  Lane CalculateTargetLane(std::unordered_map<Lane, std::vector<Car>> cars_front_back);

  double CalculateAcceleration(double cur_v_mps, double target_v_mps);
};

} // namespace car

#endif /* CAR_PATH_PLANNER_H */