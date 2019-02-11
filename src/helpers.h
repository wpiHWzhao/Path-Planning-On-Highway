//
// Created by haowei on 2/10/19.
//

#ifndef PATH_PLANNING_HELPERS_H
#define PATH_PLANNING_HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

string hasData(string s);

constexpr double pi();

double deg2rad(double);

double rad2deg(double);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double > &map_x, const vector<double > &map_y);

int NextWaypoint(double x, double y, double theta, const vector<double > &map_x,
        const vector<double > &maps_y);

vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y);

vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y);

struct CarInfo{
    double carX;
    double carY;
    double carS;
    double carD;
    double carYaw;
    double carSpeed;
};

struct MapInfo{
    vector<double > mapWayPointsX;
    vector<double > mapWayPointsY;
    vector<double > mapWayPointsS;
    vector<double > mapWayPoints_dx;
    vector<double > mapWayPoints_dy;
};


#endif //PATH_PLANNING_HELPERS_H
