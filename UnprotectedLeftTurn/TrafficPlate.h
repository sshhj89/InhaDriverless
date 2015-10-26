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

class TrafficPlate
{
private:
    Point trafficCenter;
    Point plateCenter;
    int lightsColor;
    vector<Vec3f> circles;
    vector<vector<Point>> squares;

public:
    TrafficPlate() { }
    
    void findCircles(const Mat& image);
    void drawCircles(const Mat& image);
    
    void findSquares( const Mat& image);
    void drawSquares( Mat& image);
    
    void drawTraffic(Mat& image);
    
    vector<Vec3f>& getCircles() {return this->circles;}
    vector<vector<Point>>& getSquares() { return this->squares;}
   
    double angle( Point pt1, Point pt2, Point pt0 )
    {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
    }
    

    void debugSquares( vector<vector<Point> >squares, Mat& image );
    
    
};

#endif /* defined(__UnprotectedLeftTurn__TrafficPlate__) */
