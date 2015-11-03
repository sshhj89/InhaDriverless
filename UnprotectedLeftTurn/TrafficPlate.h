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
    
    bool lightsColor; // -1: impossible, 1: green;
    bool isUnprotected;
    Mat greenHsv;
    Mat greenBin;
    
    vector<Vec3f> prevCircles;
    vector<vector<Point>> squares;
    
    vector<Vec3f> circles;
    
    bool seqImage; //check sequentialimage;

    list<Point> tpl;
    list<int> tplR;
    
    Mat origin;
    int sizeX, sizeY;
    
    double angle( Point pt1, Point pt2, Point pt0 )
    {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
    }
    
public:
    TrafficPlate() { seqImage = false; tpl.clear(); tplR.clear(); lightsColor = false;
    Mat db_original = imread("/Users/sonhojun/Downloads/signals2.png");
        sizeX = db_original.cols;
        sizeY = db_original.rows;
        
        isUnprotected = false;
    }
    
    void findCircles(const Mat& image);
    void drawCircles( Mat& image);
    void filterLights();
    void setGreenHSV(const Mat& image);
    Mat binaryOrigin();
    void splitAsGreenBin(const Mat& image);
    void findSquares( Mat& image);
    void drawSquares( Mat& image);
    
    void drawTraffic(Mat& image);
    
    vector<Vec3f>& getCircles() {return this->circles;}
    Mat& getGreenHsv() {return this->greenHsv;}
    Mat& getGreenBin() { return this->greenBin;}
    bool getLightsColor() { return this->lightsColor;}
    bool getUnprotected() { return this->isUnprotected;}
    void setGreenBin(Mat bin) {return bin.copyTo(greenBin);}
    
//    void featureDetect(Mat& image, vector<vector<Point>> squares);
    void debugSquares( vector<vector<Point> >squares, Mat& image );
    bool filterPlate(Mat& possiblePlate);
    
    
    void setOrigin(Mat ori) {ori.copyTo(origin);}
    Mat getOrigin() { return this->origin;}
    
    
};

#endif /* defined(__UnprotectedLeftTurn__TrafficPlate__) */
