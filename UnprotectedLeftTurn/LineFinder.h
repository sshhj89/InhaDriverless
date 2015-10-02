
#ifndef __LINEFINDER__
#define __LINEFINDER__

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/ml/ml.hpp>
#include <opencv2/hal.hpp>

#include <opencv2/core/ocl.hpp>
#include <cmath>

#include <iostream>
#include <cstdlib>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace std;
using namespace cv;

#define TESTNUM 50
#define FEATURES 4

class LineInfo
{
public:
    double rho;
    double slope;
    double interceptY;
    double interceptX;
    Point pt1;
    Point pt2;


    LineInfo(double r, double s, double intX, double intY, Point pt1, Point pt2)
    {
        this->rho = r;
        this->slope = s;
        this->interceptX = intX;
        this->interceptY = intY;
        this->pt1 = pt1;
        this->pt2 = pt2;

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
    
    double ArcCalc(double lx1, double lx2, double ly1, double ly2)// 각도 계산하기 입력된 직선의 xy값 2개
    {
        double inner = (lx1*lx2 + ly1*ly2);// 기본내적
        
        double i1=sqrt(lx1*lx1+ ly1*ly1); // 처음 직선의 노말라이즈 준비
        double i2=sqrt(lx2*lx2+ ly2*ly2); // 두번째 직선의 노말라이즈 준비
        
        lx1=(lx1/i1); // 각 요소를 단위 벡터로 변환한다.
        ly1=(ly1/i1);
        lx2=(lx2/i2);
        ly2=(ly2/i2);
        
        inner =(lx1*lx2 + ly1*ly2);
        double result = acos(inner)*180/CV_PI;
        // 아크 코사인을 통해 라디안을 구하고 그걸 각도로 변환하기 위해 180을 곱하고 파이로나눈다.
        if(lx1 > lx2) result=360-result;//?? 이게 아닌거 같다.
        // 사이각은 최대 0-180도 사이마 존재함으로 입력된 x값을 참조하여 360에 가까운 값으로 변환하는 과정을 거친다.
        return result ;
    }
    
    
};

class LineFinder
{
private:
    vector<Vec4i> lines;
    vector<Vec4i> positiveLines;
    vector<Vec4i> negativeLines;
    vector<Vec4i> horizontalLines;
    
    vector<LineInfo> positiveInfo;
    vector<LineInfo> negativeInfo;
    
    vector<Vec4i> arrangePositivie;
    
    double deltaRho;
    double deltaTheta;
    int minVote;
    double minLength;
    double maxGap;
    
    double perpendicular(double slope){ return (-1)/slope; }
    double calInterceptY(double pt1Y, double lineSlope, double pt1X){ return pt1Y - lineSlope * pt1X; }
    double calSlope(double pt2Y, double pt1Y, double pt2X, double pt1X) { return (pt2Y - pt1Y) / (pt2X - pt1X); }
    double calInterceptX(double interceptY, double lineSlope) { return -(interceptY / lineSlope);}
    
public:
    LineFinder() : deltaRho(1),deltaTheta(CV_PI/180), minVote(10),minLength(0.),maxGap(0.){}
    
    void setAccResolution(double r,double t) { this->deltaRho = r; this->deltaTheta = t; }
    void setMinVote(int v) { this->minVote = v; }
    void setLineLengthAndGap(double l, double g) { this->minLength = l; this->maxGap = g; }
    
    vector<Vec4i> getPositiveLines() { return this->positiveLines;}
    vector<Vec4i> getNagativeLines() { return this->negativeLines;}
    vector<Vec4i> getHorizontalLines() { return this->horizontalLines;}
    
    const vector<LineInfo>& getPositiveLinesInfo() { return this->positiveInfo;}
    const vector<LineInfo>& getNagativeLinesInfo() { return this->negativeInfo;}

    
    
    vector<Vec4i> findLines(UMat& bin);
    void filterLine(UMat& image, int myX , int myY , int pnha = 0 /*0 : positive, 1: negative 2: horizontal 3: all*/);
    
    void drawLine(UMat& image);
    void kmeansPositive();
    void kmeansNegative();
    void filterStopLine();
    
};

#endif /* defined(__UnprotectedLeftTurn__LineFinder__) */
