//
// Created by haowei on 2/10/19.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include <vector>
#include "helpers.h"
#include <math.h>

using std::vector;

class PathPlanner {
public:

    PathPlanner();

    ~PathPlanner();

    // Trajectory generator. Returns {next X vector, next Y vector}.
    vector<vector<double >> trajectoryGen (const CarInfo &, const MapInfo &);

private:
    double targetSpeed = 0.40;
};


#endif //PATH_PLANNING_PATHPLANNER_H
