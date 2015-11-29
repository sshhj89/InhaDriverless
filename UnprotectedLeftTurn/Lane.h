//
//  Lane.h
//  UnprotectedLeftTurn
//
//  Created by SonHojun on 10/15/15.
//  Copyright (c) 2015 SonHojun. All rights reserved.
//

#ifndef __LANE_H__
#define __LANE_H__
#include "MetaHeader.h"
#include <list>



#define TESTNUM 50
#define FEATURES 4

#define BOUNDSIZE 4 //make flexible
#define DIFF 25.

static double RHOMAX = 220.0 ;
static double RHOMIN = 130.0 ;

static double boundaryCountRight1 = 0;
static double boundaryCountLeft1 = 0;
static double boundaryCountRight2 = 0;
static double boundaryCountLeft2 = 0;
static double bandLeftMax = RHOMAX ;
static double bandRightMax = RHOMAX;
static double bandLeftMin = RHOMIN;
static double bandRightMin = RHOMIN;

static int BOUNDINIT = BOUNDSIZE;
static int BOUNDCOUNT = 0;

class LineInfo
{
private:  /* to do: make private */
    double rho;
    double slope;
    double interceptY;
    double interceptX;
    Point pt1;
    Point pt2;
    bool refined;
    
public:
    LineInfo(double r, double s, double intX, double intY, Point pt1, Point pt2)
    {
        this->rho = r;
        this->slope = s;
        this->interceptX = intX;
        this->interceptY = intY;
        this->pt1 = pt1;
        this->pt2 = pt2;
        this->refined = false;
    }
    
    LineInfo(){}
    
    double getRho() const { return this->rho; }
    double getSlope() const { return this->slope; }
    double getIntX() const { return this->interceptX; }
    double getIntY() const { return this->interceptY; }
    int getPoint1X() const { return pt1.x;}
    int getPoint1Y() const { return pt1.y;}
    int getPoint2X() const { return pt2.x;}
    int getPoint2Y() const { return pt2.y;}
    bool getRefined() const { return refined; }
    
    double calTheta(double lx1, double lx2, double ly1, double ly2)// 각도 계산하기 입력된 직선의 xy값 2개
    {
        double inner = (lx1*lx2 + ly1*ly2);
        
        double i1=sqrt(lx1*lx1+ ly1*ly1);
        double i2=sqrt(lx2*lx2+ ly2*ly2);
        
        inner /= (i1*i2);
        double result = acos(inner)*180/CV_PI;
        
        if(lx1 > lx2) result=360-result;
        
        return result ;
    }
};

class Lane
{
    
private:
    Mat priorImage;
    Mat currentImage;
    Mat YelloImage;
    Mat WhiteImage;
    Mat bwYellowWhiteImage;
    
    Mat laneCropImage;
    
    double deltaRho;
    double deltaTheta;
    int minVote;
    double minLength;
    double maxGap;
    
    double bkRhoPos;
    double bkRhoNeg;
    
    double currentRhoPos;
    double currentRhoNeg;
    
    vector<Vec4i> lines;
    
    vector<Vec4i> positiveLines;
    vector<LineInfo> positiveInfo;
    
    vector<Vec4i> negativeLines;
    vector<LineInfo> negativeInfo;
    
    list<double> boundRhoPos;
    list<double> boundRhoNeg;
    vector<Vec4i> horizontalLines;
    
public:
    Lane(): deltaRho(1),deltaTheta(CV_PI/180), minVote(10),minLength(0.),maxGap(0.){ this->bkRhoNeg = -0.1; this->bkRhoPos = -0.1; this->currentRhoNeg = -0.1; this->currentRhoPos = -0.1;
        this->boundRhoNeg.clear(); this->boundRhoPos.clear();
    }
    
    Mat& getPriorImage() { return this->priorImage;}
    Mat& getCurrentImage() { return this->currentImage;}
    Mat& getYellowImage() { return this->YelloImage;}
    Mat& getWhiteImage() { return this->WhiteImage;}
    Mat& getbwYellowWhiteImage() { return this->bwYellowWhiteImage;}
    Mat& getLaneCropImage() { return this->laneCropImage;}
    double getBkRhoPos() { return this->bkRhoPos;}
    double getBkRhoNeg() { return this->bkRhoNeg;}
    double getCurrentRhoPos() { return this->currentRhoPos; }
    double getCurrentRhoNeg() { return this->currentRhoNeg; }
    list<double>& getBoundRhoPos() {return this->boundRhoPos;}
    list<double>& getBoundRhoNeg() { return this->boundRhoNeg;}
    
    vector<LineInfo> getPositiveLines() {return this->positiveInfo;}
    void setPriorImage(Mat priorImage) { priorImage.copyTo(this->priorImage);}
    void setCurrentImage(Mat currentImage) { currentImage.copyTo(this->currentImage);}
    void setYellowImage(Mat yellow) { yellow.copyTo(this->YelloImage);}
    void setWhiteImage(Mat white) { white.copyTo(this->WhiteImage);}
    void setbwYellowWhiteImage(Mat bw) { bw.copyTo(this->bwYellowWhiteImage);}
    void setLaneCropImage(Mat crop) { crop.copyTo(this->laneCropImage);}
    void setCurrentRhoPos(double d) { this->currentRhoPos = d; }
    void setCurrentRhoNeg(double d) { this->currentRhoNeg =d; }
    
    void setBkRhoPos(double rho) { this->bkRhoPos = rho;}
    void setBkRhoNeg(double rho) { this->bkRhoNeg = rho; }
    bool isDetectedStopLine() { return (horizontalLines.size() == 0)? false: true;}
    
    void setBoundRhoPos(double rho){
        if(rho == -0.1) return;
        if(this->boundRhoPos.size() < BOUNDSIZE)
            this->boundRhoPos.push_back(rho);
        else
        {
            this->boundRhoPos.pop_front();
            this->boundRhoPos.push_back(rho);
        }
    }
    void setBoundRhoNeg(double rho){
        if(rho == -0.1) return;
        if(this->boundRhoNeg.size() < BOUNDSIZE)
            this->boundRhoNeg.push_back(rho);
        else
        {
            this->boundRhoNeg.pop_front();
            this->boundRhoNeg.push_back(rho);
        }
        
    }

    void findWhiteInImage(Mat& image, Mat& result);
    void findYellowInImage(Mat& image, Mat& result);
    
    void setAccResolution(double r,double t) { this->deltaRho = r; this->deltaTheta = t; }
    void setMinVote(int v) { this->minVote = v; }
    void setLineLengthAndGap(double l, double g) { this->minLength = l; this->maxGap = g; }
    
    double perpendicular(double slope){ return (-1)/slope; }
    double calInterceptY(double pt1Y, double lineSlope, double pt1X){ return pt1Y - lineSlope * pt1X; }
    double calSlope(double pt2Y, double pt1Y, double pt2X, double pt1X) { return (pt2Y - pt1Y) / (pt2X - pt1X); }
    double calInterceptX(double interceptY, double lineSlope) { return -(interceptY / lineSlope);}
    double myABSPos(double x) {
        cout<< (bandLeftMax + bandLeftMin)  /2. << " - " <<"bkPos : " <<x<<endl;
        return ((bandLeftMax + bandLeftMin) /2.  - x);}
    double myABSNeg(double x) {
        cout<< (bandRightMax + bandRightMin) /2. << " - " <<"bkNeg : " <<x<<endl;
        return ((bandRightMax + bandRightMin) /2.  - x);}
    
    void initBand()
    {
        bandLeftMax = RHOMAX ;
        bandRightMax = RHOMAX;
        bandLeftMin = RHOMIN;
        bandRightMin = RHOMIN;
    }
    
    vector<Vec4i> findLines(Mat& image, bool stop = false);
    void drawLine(Mat& image);
    void filterLine(Mat& image,double myX, double myY);
    void filterStopLine();
    
    int kmeansPositive();
    int kmeansNegative();
    
    double findMedianRho(int direction);
    
    int laneChecker();
    
};


#endif
