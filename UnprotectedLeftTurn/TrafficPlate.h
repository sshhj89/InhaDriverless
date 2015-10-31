//
//  TrafficPlate.h
//  UnprotectedLeftTurn
//
//  Created by SonHojun on 10/18/15.
//  Copyright (c) 2015 SonHojun. All rights reserved.
//

#ifndef __TRAFFIC_PLATE__
#define __TRAFFIC_PLATE__
#include "MetaHeader.h"
#include <list>

static Vec<double,4> totalDiff = 0.0;
class TrafficPlate
{
private:
    Point trafficCenter;
    Point plateCenter;
    int lightsColor;
    
    vector<Vec3f> prevCircles;
    vector<vector<Point>> squares;
    
    vector<Vec3f> curCircles;
    vector<vector<Point>> curSquares;
    
    bool seqImage; //check sequentialimage;
    vector<Vec3f> refinedCircles;
    list<Point> tpl;
    list<int> tplR;
    
    int grp[3];
    
public:
    TrafficPlate() { seqImage = false; tpl.clear(); tplR.clear();}
    
    void findCircles(const Mat& image);
    void drawCircles(const Mat& image);
    
    void findSquares( Mat& image);
    void drawSquares( Mat& image);
    
    void drawTraffic(Mat& image);
    
    vector<Vec3f>& getCurCircles() {return this->curCircles;}
    vector<vector<Point>>& getSquares() { return this->squares;}

    double angle( Point pt1, Point pt2, Point pt0 )
    {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
    }
    
    void featureDetect(Mat& image, vector<vector<Point>> squares);
    void debugSquares( vector<vector<Point> >squares, Mat& image );
    
    void filterLights();
    void setSeqImage() { seqImage = !seqImage;}
    bool getSeqImage() { return this->seqImage;}
    
};

#endif /* defined(__UnprotectedLeftTurn__TrafficPlate__) */
