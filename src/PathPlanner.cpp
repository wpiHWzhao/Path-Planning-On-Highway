//
// Created by haowei on 2/10/19.
//

#include "PathPlanner.h"

using std::vector;

PathPlanner::PathPlanner()=default;

PathPlanner::~PathPlanner()=default;

vector<vector<double >> PathPlanner::trajectoryGen(const CarInfo &carInfo, const MapInfo &mapInfo) {

    vector<double> nextXValue;
    vector<double> nextYValue;

    for (int i = 0; i < 50; ++i) {
        double nextS = carInfo.carS+targetSpeed*i;
        double nextD = 6;

        vector<double > xy = getXY(nextS,nextD,mapInfo.mapWayPointsS,mapInfo.mapWayPointsX,mapInfo.mapWayPointsY);

        nextXValue.push_back(xy[0]);
        nextYValue.push_back(xy[1]);
    }

    return {nextXValue,nextYValue};
}