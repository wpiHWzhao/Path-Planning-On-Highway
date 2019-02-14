//
// Created by haowei on 2/10/19.
//

#include "PathPlanner.h"
#include "spline.h"
using std::vector;


PathPlanner::PathPlanner()=default;

PathPlanner::~PathPlanner()=default;

vector<vector<double >> PathPlanner::trajectoryGen(const CarInfo &carInfo, const MapInfo &mapInfo,
        const vector<double> &previousPathX, const vector<double > &previousPathY) {


    // int lane = 1; // Initially in the middle lane

    // double targetSpeed = 49.5; // mph

    tk::spline s; // Spline object

    // The points that are connected with spline
    vector<double > ptsX;
    vector<double > ptsY;

    double originX = carInfo.carX;
    double originY = carInfo.carY;
    double originYaw = deg2rad(carInfo.carYaw);

    if (previousPathX.size()<2){
        double lastPointInPath_X = carInfo.carX-cos(deg2rad(carInfo.carYaw));///
        double lastPointInPath_Y = carInfo.carY-sin(deg2rad(carInfo.carYaw));///

        ptsX.push_back(lastPointInPath_X);
        ptsX.push_back(carInfo.carX);

        ptsY.push_back(lastPointInPath_Y);
        ptsY.push_back(carInfo.carY);
    } else{

        originX = previousPathX[previousPathX.size()-1];
        originY = previousPathY[previousPathY.size()-1];

        double originX_last = previousPathX[previousPathX.size()-2];
        double originY_last = previousPathY[previousPathY.size()-2];

        originYaw = atan2(originY-originY_last,originX-originX_last);

        ptsX.push_back(previousPathX[previousPathX.size()-2]);
        ptsX.push_back(previousPathX[previousPathX.size()-1]); // Last point in the previous path

        ptsY.push_back(previousPathY[previousPathY.size()-2]);
        ptsY.push_back(previousPathY[previousPathY.size()-1]); // Last point in the previous path
    }

    vector<double > nextWayPoint0 = getXY(carInfo.carS+30, (2+4*lane),
            mapInfo.mapWayPointsS, mapInfo.mapWayPointsX, mapInfo.mapWayPointsY);

    vector<double > nextWayPoint1 = getXY(carInfo.carS+60, (2+4*lane),
            mapInfo.mapWayPointsS, mapInfo.mapWayPointsX, mapInfo.mapWayPointsY);

    vector<double > nextWayPoint2 = getXY(carInfo.carS+90, (2+4*lane),
            mapInfo.mapWayPointsS, mapInfo.mapWayPointsX, mapInfo.mapWayPointsY);

    ptsX.push_back(nextWayPoint0[0]);
    ptsX.push_back(nextWayPoint1[0]);
    ptsX.push_back(nextWayPoint2[0]);

    ptsY.push_back(nextWayPoint0[1]);
    ptsY.push_back(nextWayPoint1[1]);
    ptsY.push_back(nextWayPoint2[1]);

    for (int i = 0; i < ptsX.size(); ++i) {
        // Use last point in the previous path as origin, transforms all points into local coordinate,
        // where car is heading 0 degree.

        double dx = ptsX[i]-originX;
        double dy = ptsY[i]-originY;

        ptsX[i] = dx*cos(-originYaw)-dy*sin(-originYaw);
        ptsY[i] = dx*sin(-originYaw)+dy*cos(-originYaw);
    }

    s.set_points(ptsX,ptsY);

    vector<double> nextXValue;
    vector<double> nextYValue;

    for (int j = 0; j < previousPathX.size(); ++j) {
        nextXValue.push_back(previousPathX[j]);
        nextYValue.push_back(previousPathY[j]);
    }

    double targetX = 30.0;
    double targetY = s(targetX);
    double distXY = sqrt(targetX*targetX+targetY*targetY);

    double xNextInSpline=0;

    for (int k = 0; k < 50 - previousPathX.size() ; ++k) {
        double N = (distXY/(0.02*targetSpeed/2.24));
        double xPoint = xNextInSpline+targetX/N;
        double yPoint = s(xPoint);

        xNextInSpline = xPoint;

        double x_ref = xPoint;
        double y_ref = yPoint;

        xPoint = x_ref*cos(originYaw)-y_ref*sin(originYaw);
        yPoint = x_ref*sin(originYaw)+y_ref*cos(originYaw);

        xPoint += originX;
        yPoint += originY;

        nextXValue.push_back(xPoint);
        nextYValue.push_back(yPoint);
    }

    return {nextXValue,nextYValue};

}

void PathPlanner::behaviorPlanner(const vector<vector<double >> &sensorFusion, const vector<double> &previousPathX,
        const CarInfo &carInfo ) {
    // bool safe = true;
    bool leftLane = true;
    bool rightLane = true;
    bool intentChangeLane = false;
    // double maxSpeed = 999;

    vector<double > laneCost = {0.0,0.1,0.2};


    for (int i = 0; i < sensorFusion.size(); ++i) {
        double d = sensorFusion[i][6];
        double vx = sensorFusion[i][3];
        double vy = sensorFusion[i][4];
        double speed = sqrt(vx*vx+vy*vy);
        double s = sensorFusion[i][5];
        bool thisLeftFlag = true;
        bool thisRightFlag = true;



        double sNear = s+ previousPathX.size()*0.02*speed;
        if (d<12 && d>0) {
            // std:: cout << "this is a car"<<std:: endl;
            // std:: cout << "d is "<< d<< std::endl;
            // std:: cout << "dist is "<< fabs(sNear - carInfo.carS)<<std::endl;
            if (d < (2 + 4 * (lane - 1) + 2) && d > (2 + 4 * (lane - 1) - 2) && fabs(sNear - carInfo.carS) < 15 ) {///15
                thisLeftFlag = false;
                // std::cout<< "left is clear" << std::endl;
            } else if (d < (2 + 4 * (lane + 1) + 2) && d > (2 + 4 * (lane + 1) - 2) && fabs(sNear - carInfo.carS) < 15) {
                thisRightFlag = false;
                // std::cout<< "right is clear" << std:: endl;
            }
        }

        if (d<(2+4*lane+2) && d>(2+4*lane-2)){
            if ((carInfo.carS<sNear )&& (sNear-carInfo.carS)<30){
                intentChangeLane = true;

                // maxSpeed = speed;

                // std::cout<< "too close !!!!"<<std::endl;
            }
        }
        leftLane = leftLane && thisLeftFlag;
        rightLane = rightLane && thisRightFlag;

    }

    //std::cout << "left lane"<< leftLane<<std::endl;
    //std::cout << "right lane"<< rightLane << std:: endl;
    //std::cout<< "intent to change lane" << intentChangeLane<<std::endl;

    if (lane == 1) {
        if (leftLane && intentChangeLane) {
            lane -= 1;
        } else if (rightLane && intentChangeLane) {
            lane += 1;
        } else if (!leftLane && !rightLane && intentChangeLane) {
            targetSpeed -= 0.224 * 3; /// 2
        } else if (targetSpeed <= 49.5) {
            targetSpeed += 0.224*3;
        }
    } else if (lane==0){
        if (rightLane && intentChangeLane) {
            lane += 1;
        } else if ( !rightLane && intentChangeLane) {
            targetSpeed -= 0.224 * 3;
        } else if (targetSpeed <= 49.5) {
            targetSpeed += 0.224*3;
        }
    } else if (lane == 2){
        if (leftLane && intentChangeLane) {
            lane -= 1;
        } else if (!leftLane && intentChangeLane) {
            targetSpeed -= 0.224 * 3;
        } else if (targetSpeed <= 49.5) {
            targetSpeed += 0.224 * 3;
        }
    }


//    bool changeLane = safe && intentChangeLane;
//
//    std::cout<<"safe is "<< safe<<std::endl;
//    std::cout<<"intentChangeLane is "<< intentChangeLane<<std::endl;
//
//    if (!safe && intentChangeLane){
//        targetSpeed -= 0.224*3;
//    } else if(changeLane){
//        calculateCost(laneCost);
//    } else if (targetSpeed<=49.5){
//        targetSpeed += 0.224;
//    }




//    if (changeLane){
//        targetSpeed -= 0.224*2;
//        // std::cout<< "reducing speed"<<std::endl;
//    } else if (targetSpeed<=49.5){
//        targetSpeed += 0.224;
//        // std::cout<< "increasing speed"<<std::endl;
//    }

//    if (targetSpeed>25){
//        calculateCost(laneCost);
//    }

}

void PathPlanner::calculateCost(vector<double > &laneCost ) {

    // int preLane = lane;

//    if(tooClose){
//        laneCost[lane]+=0.5;
//        laneCost[abs(lane-2)] +=0.3; // We always punish go across 2 lanes in a single change
//    }

    laneCost[lane]+=0.5;
    laneCost[abs(lane-2)] +=0.3; // We always punish go across 2 lanes in a single change

    // int intantLane = int(std::min_element(laneCost.begin(),laneCost.end())-laneCost.begin());

    lane = int(std::min_element(laneCost.begin(),laneCost.end())-laneCost.begin());


}