//
// Created by Haowei Zhao on 2/10/19.
//

#include "PathPlanner.h"
#include "spline.h"
using std::vector;


PathPlanner::PathPlanner()=default;

PathPlanner::~PathPlanner()=default;

vector<vector<double >> PathPlanner::trajectoryGen(const CarInfo &carInfo, const MapInfo &mapInfo,
        const vector<double> &previousPathX, const vector<double > &previousPathY) {

    /** This is the path planner. Based on the decision of the Behaviour Planner, we can project some points on the target
     * lane, then use spline to connect them with points on our current lane to create a smooth path.
     */


    tk::spline s; // Spline object

    // The points to be connected with spline
    vector<double > ptsX;
    vector<double > ptsY;

    // We set the origin of the coordination on our current car position if there is almost no previous path
    double originX = carInfo.carX;
    double originY = carInfo.carY;
    double originYaw = deg2rad(carInfo.carYaw);

    if (previousPathX.size()<2){
        // If the previous path is almost empty, we calculate the previous point
        double lastPointInPath_X = carInfo.carX-cos(deg2rad(carInfo.carYaw));
        double lastPointInPath_Y = carInfo.carY-sin(deg2rad(carInfo.carYaw));

        // Put previous point and current point into the vector
        ptsX.push_back(lastPointInPath_X);
        ptsX.push_back(carInfo.carX);

        ptsY.push_back(lastPointInPath_Y);
        ptsY.push_back(carInfo.carY);
    } else{

        // If we have a previous path, the we set origin at the last point in previous path
        originX = previousPathX[previousPathX.size()-1];
        originY = previousPathY[previousPathY.size()-1];

        // Second to last point in the previous path
        double originX_last = previousPathX[previousPathX.size()-2];
        double originY_last = previousPathY[previousPathY.size()-2];

        originYaw = atan2(originY-originY_last,originX-originX_last);

        ptsX.push_back(previousPathX[previousPathX.size()-2]);
        ptsX.push_back(previousPathX[previousPathX.size()-1]); // Last point in the previous path

        ptsY.push_back(previousPathY[previousPathY.size()-2]);
        ptsY.push_back(previousPathY[previousPathY.size()-1]); // Last point in the previous path
    }

    // Project some points on the intended lane
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

    // Create spline
    s.set_points(ptsX,ptsY);

    vector<double> nextXValue;
    vector<double> nextYValue;

    // We should follow all the points left in the previous path
    for (int j = 0; j < previousPathX.size(); ++j) {
        nextXValue.push_back(previousPathX[j]);
        nextYValue.push_back(previousPathY[j]);
    }

    // Here we want to pick some points on the spline so we can follow.
    // We can do this by separating the spline into N pieces, then put a point on the separating position
    double targetX = 30.0;
    double targetY = s(targetX);
    double distXY = sqrt(targetX*targetX+targetY*targetY);

    double xNextInSpline=0;

    for (int k = 0; k < 50 - previousPathX.size() ; ++k) {
        double N = (distXY/(0.02*targetSpeed/2.24));
        double xPoint = xNextInSpline+targetX/N; // X of next separating point
        double yPoint = s(xPoint); // Y of next separating point

        xNextInSpline = xPoint;

        double x_ref = xPoint;
        double y_ref = yPoint;

        // We transform back from local coordinate to world coordinate
        xPoint = x_ref*cos(originYaw)-y_ref*sin(originYaw);
        yPoint = x_ref*sin(originYaw)+y_ref*cos(originYaw);

        // Add this position to the origin to get next position in the world frame
        xPoint += originX;
        yPoint += originY;

        nextXValue.push_back(xPoint);
        nextYValue.push_back(yPoint);
    }

    return {nextXValue,nextYValue};

}

void PathPlanner::behaviorPlanner(const vector<vector<double >> &sensorFusion, const vector<double> &previousPathX,
        const CarInfo &carInfo ) {

    /** This behavior planner includes 2 setup - one is an Aggressive Driver, another one is a Gentle Driver.
     *
     * --Aggressive Driver would try to follow the lane that has the highest speed, which is defined by the speed of
     * the slowest car in that lane and within sensor range. This driver is capable of making quick maneuvers, at the
     * cost of possible large jerks. It prefers left lane over middle lane, and middle lane over right lane.
     *
     * --Gentle driver would change lane only when it is necessary, e.g. when there is a car in front of it and
     * it is safe to do so. It also prefers left lane over right lane.
     * */
/*
    //-------------------------------------------------------------------------------------------
    /// Aggressive Driver below

    // Each initial lane speed is very large, but left lane is the largest because this driver prefers left lane.
    vector<double > laneSpeed = {999,888,777};

    bool leftLane = true; // Left lane flag. True if the left lane is clear
    bool middleLane = true; // Middle lane flag. True if the middle lane is clear
    bool rightLane = true; // Right lane flag. True if the right lane is clear.

    // For each car with in our sensor range:
    for (int i = 0; i < sensorFusion.size(); ++i) {
        double d = sensorFusion[i][6]; // Frenet d value of this car
        double vx = sensorFusion[i][3]; // Cartesian x velocity
        double vy = sensorFusion[i][4]; // Cartesian y velocity
        double speed = sqrt(vx*vx+vy*vy); // Speed
        double s = sensorFusion[i][5]; // Frenet s value of this car
        int thisCarLane; // The lane that this car is in
        bool thisLeftLane = true; // True if this car is not blocking the left lane
        bool thisMiddleLane = true; // True if this car is not blocking the middle lane
        bool thisRightLane = true; // True if this car is not blocking the right lane

        double sNear = s+ previousPathX.size()*0.02*speed; // Predict where this car would be in the next frame

        // Check which lane the car is in
        if (d<4 && d>0) thisCarLane =0;
        else if (d>4 && d<8) thisCarLane =1;
        else if (d>8 && d<12) thisCarLane =2;
        else continue;

        // If the car is within the range of interest, and is slower then the last minimum speed, then we set the lane
        // speed as this car speed. Because the car is driving forward fast, we are more care about cars in front of
        // us.
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

        leftLane = leftLane && thisLeftLane; // True if all cars in the left lane are not blocking it.
        middleLane = middleLane && thisMiddleLane; // True if all cars in the middle lane are not blocking it.
        rightLane = rightLane && thisRightLane; // True if all cars in the right lane are not blocking it.

    }

    int intendLane = int(std::max_element(laneSpeed.begin(), laneSpeed.end())-laneSpeed.begin());

    ///  You can uncomment the following lines to see how the car is making decisions.
    // std::cout<< "intendlane: "<<intendLane<<std::endl;
    // std::cout<<"leftlane speed "<< laneSpeed[0]<<std::endl;
    // std::cout << "middlelane speed "<< laneSpeed[1]<<std::endl;
    // std::cout<< "rightlane speed "<< laneSpeed[2]<<std::endl;

    // Decision tree:
    switch (lane){
        case 0:{ // If we are currently in the left lane
            switch (intendLane){
                case 0:{ // If we want to keep lane
                    if(leftLane && targetSpeed<45){ // if left lane is clear and we are under speed limit
                        targetSpeed +=0.224*3; // Speed up
                    } else{ // If the left lane is blocked , or we are too fast
                        targetSpeed -=0.224*5; // Slow down
                    }
                }
                    break;
                case 1:{ // If we want to go to the middle lane
                    if (middleLane && targetSpeed<45){
                        lane = 1; // Change lane
                        targetSpeed +=0.224*3;
                    } else{
                        targetSpeed -=0.224*5;
                    }
                }
                    break;
                case 2:{ // If we want to go to the right lane
                    if (middleLane && rightLane && targetSpeed<45){ // If both the middle lane and right lane are not blocked
                        lane = 2; // Change lane
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
                        targetSpeed += 0.224*3;
                    } else if(leftLane && !middleLane && targetSpeed<45){
                        targetSpeed += 0.224*3;
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
*/
    //------------------------------------------------------------------------------------------------------

    /// Gentle Driver below

    // The flag of the current lane on the left, true if this lane is not blocked.
    // NOTE: This is not the left lane on the map.
    bool leftLane = true;
    // The flag of the current lane on the right, true if this lane is not blocked.
    // NOTE: This is not he right lane on the map
    bool rightLane = true;
    bool intentChangeLane = false; // True if we want to change lane


    // Check each car within our sensor range
    for (int i = 0; i < sensorFusion.size(); ++i) {
        double d = sensorFusion[i][6];
        double vx = sensorFusion[i][3];
        double vy = sensorFusion[i][4];
        double speed = sqrt(vx*vx+vy*vy);
        double s = sensorFusion[i][5];
        bool thisLeftFlag = true; // True if lane on the left is not blocked
        bool thisRightFlag = true; // True if lane on the right is not blocked

        double sNear = s+ previousPathX.size()*0.02*speed; // Predict where the car is in the next frame
        if (d<12 && d>0) { // The car should be on our side of the road
            if (d < (2 + 4 * (lane - 1) + 2) && d > (2 + 4 * (lane - 1) - 2) && fabs(sNear - carInfo.carS) < 15 ) {
            // If the car is on our left and very close, then is not safe to change lane
                thisLeftFlag = false;
            } else if (d < (2 + 4 * (lane + 1) + 2) && d > (2 + 4 * (lane + 1) - 2) && fabs(sNear - carInfo.carS) < 15) {
            // If the car is on our right and very close, then is not safe to change lane
                thisRightFlag = false;
            }
        }

        if (d<(2+4*lane+2) && d>(2+4*lane-2)){ // If there is a car in our lane
            if ((carInfo.carS<sNear )&& (sNear-carInfo.carS)<30){ // And it is in front of us and blocking our way
                intentChangeLane = true; // Want to change lane
            }
        }
        leftLane = leftLane && thisLeftFlag; // True if all cars on the left are not blocking our way
        rightLane = rightLane && thisRightFlag; // True if all cars on the right are not blocking our way

    }

    // Decision tree:
    if (lane == 1) { // If we are in the middle lane
        if (leftLane && intentChangeLane) { // The driver always prefers to change left if it is safe to do so
            lane -= 1;
        } else if (rightLane && intentChangeLane) {
            lane += 1;
        } else if (!leftLane && !rightLane && intentChangeLane) { // If all ways are blocked, then we slow down
            targetSpeed -= 0.224;
        } else if (targetSpeed <= 49.5) { // If there is no cars blocking our way, speed up
            targetSpeed += 0.224;
        }
    } else if (lane==0){
        if (rightLane && intentChangeLane) {
            lane += 1;
        } else if ( !rightLane && intentChangeLane) {
            targetSpeed -= 0.224;
        } else if (targetSpeed <= 49.5) {
            targetSpeed += 0.224;
        }
    } else if (lane == 2){
        if (leftLane && intentChangeLane) {
            lane -= 1;
        } else if (!leftLane && intentChangeLane) {
            targetSpeed -= 0.224;
        } else if (targetSpeed <= 49.5) {
            targetSpeed += 0.224;
        }
    }


}
