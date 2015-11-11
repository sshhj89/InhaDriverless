//
//  UV.h
//  UnprotectedLeftTurn
//
//  Created by SonHojun on 10/18/15.
//  Copyright (c) 2015 SonHojun. All rights reserved.
//

#ifndef __UV_H__
#define __UV_H__

#include "MetaHeader.h"
#include "Lane.h"
#include "TrafficPlate.h"

#define LINEDETECTION 0
#define STOPLINEDETECTED 1
#define TRAFFICLIGHTSDETECTED 2
#define UNPROTECTEDDETECTED 3

class UVCar
{
private:
    Lane lf;
    TrafficPlate tp;
    
    int state; //0: line detecting 1: detected stop Line
    bool possibleLine;
    int onLane;

    
 
public:
    UVCar() { lf = Lane();  possibleLine = false; onLane = 0; state = LINEDETECTION; }
    
    Lane& getLaneFinder() { return this->lf;}
    TrafficPlate& getTrafficFinder() { return this->tp; }
    
    void setState(int s) { this->state =s;}
  
    int getState() { return this->state;}
    
    void modifyOnLane(int l);
    bool getPossibleLine() { return this->possibleLine;}
    int getOnLane() { return this->onLane;}

    void setPossibleLine(bool b ) { this->possibleLine = b;}
    void setOnLane(int l) { this->onLane = l;}
   
    
};

#endif /* defined(__UnprotectedLeftTurn__UV__) */
