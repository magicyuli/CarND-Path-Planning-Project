#ifndef CAR_UTILS_H
#define CAR_UTILS_H

#include <vector>

using std::vector;
using std::pair;

namespace car {

namespace utils {

constexpr double Pi();

// For converting back and forth between radians and degrees.
double DegToRad(double x);

double RadToDeg(double x);

double MphToKph(double mph);

double MphToMps(double mph);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> GetXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform points from map coordinate to car coordinate
pair<vector<double>, vector<double>> MapXYToCarXY(vector<double> map_x, vector<double> map_y, double ref_x, double ref_y, double ref_yaw);

// Transform points from map coordinate to car coordinate
pair<vector<double>, vector<double>> CarXYToMapXY(vector<double> car_x, vector<double> car_y, double ref_x, double ref_y, double ref_yaw);

template<typename T>
void PrintVec(vector<T> vec1, vector<T> vec2);

template<typename T>
void PrintVec(vector<T> vec);

} // namespace utils

} // namespace car

#endif /* CAR_UTILS_H */