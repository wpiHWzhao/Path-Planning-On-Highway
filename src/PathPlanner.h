//
// Created by Haowei Zhao on 2/10/19.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include <vector>
#include "helpers.h"
#include <math.h>
#include <iostream>
#include <algorithm>

using std::vector;

class PathPlanner {
public:

    PathPlanner();

    ~PathPlanner();

    // Trajectory generator. Returns {next X vector, next Y vector}.
    vector<vector<double >> trajectoryGen (const CarInfo &, const MapInfo &,
            const vector<double > &, const vector<double> &);

    // Behavior generator. Would change speed and lane based on environment information
    void behaviorPlanner(const vector<vector<double >> &, const vector<double > &, const CarInfo & );

private:
    double targetSpeed = 0;
    int lane = 1; // Initial lane
};



#endif //PATH_PLANNING_PATHPLANNER_H
