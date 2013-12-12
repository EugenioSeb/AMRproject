#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <iostream>
#include <vector>
#include <ctime>
#include <math.h>
#include <sys/time.h>
#include <thread>
#include "utils.h"
#include "robtimer.h"
extern "C" {
#include "remoteApi/extApi.h"
#include "remoteApi/v_repConst.h"
/*	#include "extApiCustom.h" if you wanna use custom remote API functions! */
}

using namespace std ;


class obstacle
{
private:
    thread _motion;
    int _clientId;
    simxInt _obsHandle;
    simxFloat _position[];
    simxFloat obsTrajectory(vector<float> &p_obs, double &time, vector<float> &start, vector<float> &end);
    void move(vec start, vec end);
public:
    vector<vec> getBoundingBox();
    obstacle(const char* Name);
    void startMove(vec start, vec end);
    float _directionTheta;
    float _currentModule;
    //void stopMove();

    vec getDirection();
};

#endif // OBSTACLE_H
