#include "MetaHeader.h"
#include "Lane.h"

#define GAP 100
using namespace std;

static double myX = cvRound((LANECROP_W + GAP)/2);
static double myY = cvRound(LANECROP_H + GAP - 50);

int main()
{
    //ocl::setUseOpenCL(true);
    
    VideoCapture video("/Users/sonhojun/Downloads/data/changeline4.mp4");
    
    if(!video.isOpened())
    {
        cout<<"video open error"<<endl;
        return 0 ;
    }
    
    Lane lf = Lane();
    Mat element(1,1,CV_8U,Scalar(200));
    
    
    Mat temp;
    Mat laneCrop;
    int chlanecount = 0;
    while (1)
    {
       
        video.read(temp);
        
        if(temp.channels() == 3)
            cvtColor(temp, temp, CV_BGR2HSV);
        lf.setPriorImage(temp);
        temp.release();
        
        video.read(temp);
        if(temp.channels() == 3)
            cvtColor(temp, temp, CV_BGR2HSV);
        
        lf.setCurrentImage(temp);
        
        laneCrop = lf.getPriorImage()(Rect(LANECROP_X - GAP,LANECROP_Y - GAP,LANECROP_W + GAP,LANECROP_H + GAP - 50));
        
        lf.findWhiteInImage(laneCrop, temp);
        medianBlur(temp, temp, 3);
        lf.setWhiteImage(temp);
        
        lf.findYellowInImage(laneCrop, temp);
        lf.setYellowImage(temp);
        temp.release();
        bitwise_or(lf.getYellowImage(), lf.getWhiteImage(), temp);
        lf.setbwYellowWhiteImage(temp);
        temp.release();
        imshow("bw",lf.getbwYellowWhiteImage());
        imshow( "white", lf.getWhiteImage());
//      imshow("yellow",lf.getYellowImage());
        
        lf.setLineLengthAndGap(40,30);  //line pixel and gap
        lf.setMinVote(60);
        
        Canny(lf.getbwYellowWhiteImage(),temp, 100,350,5);
//      imshow("edge", temp);
        lf.findLines(temp,true);
        lf.filterLine(temp,myX,myY);
        lf.filterStopLine();
        int kmeansResultPos = lf.kmeansPositive();
        
        double r = lf.findMedianRho(1);
        cout<<"Rhopos: " << r<<", ";
        if(lf.getBkRhoPos() == -0.1 || r == -0.1)
            lf.setBkRhoPos(r);
        else
        {
            lf.setBkRhoPos(lf.getCurrentRhoPos());
            lf.setBoundRhoPos(lf.getCurrentRhoPos());
            lf.setCurrentRhoPos(r);
        }
    
        r = lf.findMedianRho(2);
        cout<<"Rhoneg: "<<r<<endl;
        
        if(lf.getBkRhoNeg() == -0.1 || r == -0.1)
            lf.setBkRhoNeg(r);
        else
        {
            lf.setBkRhoNeg(lf.getCurrentRhoNeg());
            lf.setBoundRhoNeg(lf.getCurrentRhoNeg());
            lf.setCurrentRhoNeg(r);
        }
        
        lf.drawLine(laneCrop);
        line(laneCrop,Point(myX,myY),Point(myX,0),Scalar(0,0,255),2,CV_AA);
        
        int chLane = lf.laneChecker();
        if(chLane != 0 )
            chlanecount +=chLane;
        string text = "current state: " + to_string(chlanecount);
        
        putText(laneCrop, text, Point(10, 130),1, 2.0, cvScalar(255, 255, 255, 0));
        
        imshow("line",laneCrop);
        cout<<"chlane : " << chlanecount << endl;
        cout<<"======================"<<endl;
        cvWaitKey(10);
        temp.release();
        laneCrop.release();
    }

    return 0;
}