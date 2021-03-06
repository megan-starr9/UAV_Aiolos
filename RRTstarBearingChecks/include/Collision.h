//
// Collision.h
// UAV_Aiolos
//
// Created by Ryan Dutoit on 6/7/12.
// Copyright 2012 __MyCompanyName__. All rights reserved.
//
#include <vector>
#include "UAV.h"
class Collision {
private:
    double time;
    double x;
    double y;
public:
    std::vector<UAV*> uavs;
    Collision(double time, double x, double y);
    Collision();
    double getX();
    double getY();
    double getTime();
    void setX(double x);
    void setY(double y);
    void setTime(double time);
};
