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

    
    vector<double > laneSpeed = {999,888,777};
    bool leftLane = true;
    bool middleLane = true;
    bool rightLane = true;

    for (int i = 0; i < sensorFusion.size(); ++i) {
        double d = sensorFusion[i][6];
        double vx = sensorFusion[i][3];
        double vy = sensorFusion[i][4];
        double speed = sqrt(vx*vx+vy*vy);
        double s = sensorFusion[i][5];
        int thisCarLane;
        bool thisLeftLane = true;
        bool thisMiddleLane = true;
        bool thisRightLane = true;

        double sNear = s+ previousPathX.size()*0.02*speed;

        if (d<4 && d>0) thisCarLane =0;
        else if (d>4 && d<8) thisCarLane =1;
        else if (d>8 && d<12) thisCarLane =2;
        else continue;

        if (speed<laneSpeed[thisCarLane]
        && sNear-carInfo.carS < 20 && sNear-carInfo.carS > -10) laneSpeed[thisCarLane] = speed;

        if (sNear-carInfo.carS < 10 && sNear-carInfo.carS>-5){ // If the car is in dangerous distance
            switch (thisCarLane){
                case 0: thisLeftLane= false;
                    break;
                case 1: thisMiddleLane= false;
                    break;
                case 2: thisRightLane = false;
                    break;
            }
        }

        leftLane = leftLane && thisLeftLane; // True if left lane is clear
        middleLane = middleLane && thisMiddleLane;
        rightLane = rightLane && thisRightLane;

    }

    int intendLane = int(std::max_element(laneSpeed.begin(), laneSpeed.end())-laneSpeed.begin());

    std::cout<< "intendlane: "<<intendLane<<std::endl;
    std::cout<<"leftlane speed"<< laneSpeed[0]<<std::endl;
    std::cout << "middlelane speed"<< laneSpeed[1]<<std::endl;
    std::cout<< "rightlane speed"<< laneSpeed[2]<<std::endl;


    switch (lane){
        case 0:{
            switch (intendLane){
                case 0:{
                    if(leftLane && targetSpeed<45){
                        targetSpeed +=0.224*3;
                    } else{
                        targetSpeed -=0.224*5;
                    }
                }
                    break;
                case 1:{
                    if (middleLane && targetSpeed<45){
                        lane = 1;
                        targetSpeed +=0.224*3;
                    } else{
                        targetSpeed -=0.224*5;
                    }
                }
                    break;
                case 2:{
                    if (middleLane && rightLane && targetSpeed<45){
                        lane = 2;
                        targetSpeed +=0.224*3;
                    } else{
                        targetSpeed -=0.224*5;
                    }
                }
                    break;
            }
        }
            break;

        case 1:{
            switch (intendLane){
                case 0:{
                    if(leftLane && targetSpeed<45){
                        lane =0;
                        targetSpeed +=0.224*3;
                    } else{
                        targetSpeed -=0.224*3;
                    }
                }
                    break;

                case 1:{
                    if (middleLane && targetSpeed<45){
                        targetSpeed +=0.224*3;
                    } else{
                        targetSpeed -=0.224*3;
                    }
                }
                    break;

                case 2:{
                    if (rightLane && targetSpeed<45){
                        lane = 2;
                        targetSpeed +=0.224*3;
                    } else{
                        targetSpeed -=0.224*3;
                    }
                }
                    break;
            }
        }
            break;

        case 2:{
            switch (intendLane){
                case 0:{
                    if(leftLane && middleLane && targetSpeed<45){
                        lane =0;
                        targetSpeed +=0.224*3;
                    } else{
                        targetSpeed -=0.224*3;
                    }
                }
                    break;
                case 1:{
                    if (middleLane && targetSpeed<45){
                        lane = 1;
                        targetSpeed +=0.224*3;
                    } else{
                        targetSpeed -=0.224*3;
                    }
                }
                    break;
                case 2:{
                    if (rightLane && targetSpeed<45){
                        targetSpeed +=0.224*3;
                    } else{
                        targetSpeed -=0.224*3;
                    }
                }
                    break;
            }
        }
            break;
    }






/*
    bool leftLane = true;
    bool rightLane = true;
    bool intentChangeLane = false;

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
            if (d < (2 + 4 * (lane - 1) + 2) && d > (2 + 4 * (lane - 1) - 2) && fabs(sNear - carInfo.carS) < 15 ) {
                thisLeftFlag = false;
            } else if (d < (2 + 4 * (lane + 1) + 2) && d > (2 + 4 * (lane + 1) - 2) && fabs(sNear - carInfo.carS) < 15) {
                thisRightFlag = false;
            }
        }

        if (d<(2+4*lane+2) && d>(2+4*lane-2)){
            if ((carInfo.carS<sNear )&& (sNear-carInfo.carS)<30){
                intentChangeLane = true;
            }
        }
        leftLane = leftLane && thisLeftFlag;
        rightLane = rightLane && thisRightFlag;

    }

    if (lane == 1) {
        if (leftLane && intentChangeLane) {
            lane -= 1;
        } else if (rightLane && intentChangeLane) {
            lane += 1;
        } else if (!leftLane && !rightLane && intentChangeLane) {
            targetSpeed -= 0.224 * 3;
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
*/

}
