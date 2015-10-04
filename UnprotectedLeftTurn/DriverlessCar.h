#ifndef __DRIVERLESSCAR__
#define __DRIVERLESSCAR__


#include "LineFinder.h"

#define LANECROP_X 500
#define LANECROP_Y 460
#define LANECROP_W 360
#define LANECROP_H 180

class DriverlessCar
{
private:
    int centerX;
    int centerY;
    int whichLane;  //1, 2, 3 : means each lane
    double bandRight;
    double bandLeft;
    double bkBandRight;
    double bkBandLeft;
    int state; // 0: possible line + not horizontal line
               // 1: horizontal line detected.
               // 2: detected traffic lights
               // 3: detected unprotected left turn traffic signal
    
    double endX;
    double endY;
    LineInfo criteriLine;
    double theta;
    
    vector<Vec3f> circles;
    
    double angle( Point pt1, Point pt2, Point pt0 )
    {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
    }
    
    
public:
    DriverlessCar()
    {
        this->endX = (LANECROP_W) / 2. ;
        this->endY = 10.;
        this->criteriLine = LineInfo(0.0, 0.0, 0.0, 0.0, Point(this->endX,this->endY),
                                     Point(this->centerX,this->centerY));
        this->theta = 0.0;
        this->bandRight = -1.;
        this->bandLeft = -1. ;
        this->circles.clear();
    }
    
    LineFinder finder;
    
    void findCircles(const UMat& image, vector<Vec3f>& circles);
    void drawCircles(const UMat& image, const vector<Vec3f>& circles);
    
    void findSquares( const UMat& image, vector<vector<Point> >& squares );
    void drawSquares( UMat& image, const vector<vector<Point> >& squares );
    
    void drawTraffic(const UMat& image, const vector<vector<Point> >& squares,const vector<Vec3f>& circles);
    
    int getCenterX(){return this->centerX;}
    int getCenterY(){return this->centerY;}
    int getWhichLane(){return this->whichLane;}
    double getBandRight(){return this->bandRight;}
    double getBandLeft(){return this->bandLeft;}
    int getState() { return this->state;}
    double getEndX() { return this->endX;}
    double getEndY() { return this->endY;}
    LineInfo getCriteriaLine() { return this->criteriLine; }
    double getBkBandLeft() { return this->bkBandLeft; }
    double getBkBandRight() { return this->bkBandRight;}
    
    void setCenterX(int centerX){ this->centerX = centerX;}
    void setCenterY(int centerY){ this->centerY = centerY;}
    void setWhichLane(int whichLane){ this->whichLane = whichLane;}
    void setBandRight(double bandRight){ this->bandRight = bandRight;}
    void setBandLeft(double bandLeft){ this->bandLeft = bandLeft;}
    void setState(int state) { this->state = state;}
    void setBkBandRight(double d) { this->bkBandRight = d; }
    void setBkBandLeft(double d) { this->bkBandLeft = d; }
    
    int laneChecker(bool rfPosi, bool rfNega);
   };

#endif 