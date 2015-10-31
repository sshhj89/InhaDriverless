#include "MetaHeader.h"
#include "UV.h"
#include <future>

#define GAP 0
using namespace std;

static double myX = cvRound((LANECROP_W + GAP)/2);
static double myY = cvRound(LANECROP_H + GAP /*-50*/);

void putTextonImage(string t, const Mat& image)
{
    string text = "current state: " + t;
    putText(image, text, Point(10, 130),1, 1.0, cvScalar(255, 255, 255, 0));
}

/*
 square 에서 비보호 시그널이 발견 되면 이후의 계싼은 하지 않는다.
 */
 

int main()
{
    //ocl::setUseOpenCL(true);
    
    VideoCapture video("/Users/sonhojun/Downloads/data/plate3.mp4");
    
    if(!video.isOpened())
    {
        cout<<"video open error"<<endl;
        return 0 ;
    }
    
    Mat temp;
    Mat origin;
    UVCar car = UVCar();
    int chLane;
    
    car.setState(LINEDETECTION);
    
    Mat grayImage1, grayImage2;
    Mat differenceImage;
    
    while (1)
    {
        
        video.read(temp);
        
        if(temp.channels() == 3)
            cvtColor(temp, temp, CV_BGR2HSV);
        car.getLaneFinder().setPriorImage(temp);

        video.read(temp);
        temp.copyTo(origin);
        
        if(temp.channels() == 3)
            cvtColor(temp, temp, CV_BGR2HSV);
        
        car.getLaneFinder().setCurrentImage(temp);
        
       
        
        if(car.getState() == LINEDETECTION)
        {
            Mat laneCrop = car.getLaneFinder().getPriorImage()(Rect(LANECROP_X - GAP,LANECROP_Y - GAP,LANECROP_W + GAP,LANECROP_H + GAP /*- 50*/));
            
            car.getLaneFinder().findWhiteInImage(laneCrop, temp);
            medianBlur(temp, temp, 3);
            car.getLaneFinder().setWhiteImage(temp);
            
            car.getLaneFinder().findYellowInImage(laneCrop, temp);
            car.getLaneFinder().setYellowImage(temp);
            bitwise_or(car.getLaneFinder().getYellowImage(), car.getLaneFinder().getWhiteImage(), temp);
            car.getLaneFinder().setbwYellowWhiteImage(temp);
            
            car.getLaneFinder().setLineLengthAndGap(40,30);  //line pixel and gap
            car.getLaneFinder().setMinVote(60);
            
            if(car.getPossibleLine() == false)
            {
                Canny(car.getLaneFinder().getYellowImage(),temp, 100,350,5);
                car.getLaneFinder().findLines(temp,true);
                car.getLaneFinder().filterLine(temp,myX,myY);
                int kmeansResultPos = car.getLaneFinder().kmeansPositive();
                
                if(kmeansResultPos == 3)
                {
                    car.setOnLane(0);
                    car.setPossibleLine(true);
                    cvWaitKey(1000);
                }
            }
            
            Canny(car.getLaneFinder().getbwYellowWhiteImage(),temp, 100,350,5);
            car.getLaneFinder().findLines(temp,true);
            car.getLaneFinder().filterLine(temp,myX,myY);
            car.getLaneFinder().filterStopLine();
           
            if(car.getLaneFinder().isDetectedStopLine())
            {
                if(car.getPossibleLine() != true)
                    continue;
                
                car.setState(STOPLINEDETECTED);

            }
            
            car.getLaneFinder().kmeansPositive();
            
            double r = car.getLaneFinder().findMedianRho(1);
            if(car.getLaneFinder().getBkRhoPos() == -0.1 || r == -0.1)
                car.getLaneFinder().setBkRhoPos(r);
            else
            {
                car.getLaneFinder().setBkRhoPos(car.getLaneFinder().getCurrentRhoPos());
                car.getLaneFinder().setBoundRhoPos(car.getLaneFinder().getCurrentRhoPos());
                car.getLaneFinder().setCurrentRhoPos(r);
            }
            
            r = car.getLaneFinder().findMedianRho(2);
            
            if(car.getLaneFinder().getBkRhoNeg() == -0.1 || r == -0.1)
                car.getLaneFinder().setBkRhoNeg(r);
            else
            {
                car.getLaneFinder().setBkRhoNeg(car.getLaneFinder().getCurrentRhoNeg());
                car.getLaneFinder().setBoundRhoNeg(car.getLaneFinder().getCurrentRhoNeg());
                car.getLaneFinder().setCurrentRhoNeg(r);
            }
            
            car.getLaneFinder().drawLine(laneCrop);
            line(laneCrop,Point(myX,myY),Point(myX,0),Scalar(0,0,255),2,CV_AA);
            
            chLane = car.getLaneFinder().laneChecker();
            car.setOnLane(chLane);
            car.modifyOnLane(chLane);
           
            putTextonImage(to_string(chLane), laneCrop);
            //imshow("line",laneCrop);
            
            cout<<"currentLane : " << car.getOnLane() << endl;
            cout<<"possible line: " << car.getPossibleLine() << endl;
            cout<<"========================================="<<endl;
            laneCrop.release();
            temp.release();
        }
        
        if(car.getState() == STOPLINEDETECTED)
        {
            vector<vector<Point>> contours;
            vector<vector<Point>> newSquares;
            vector<vector<Point>> newCircles;
            vector<Point> heightwidth;
            Mat plateCrop;
            
         
            
            if(car.getTrafficFinder().getSeqImage() == false)
            {
                plateCrop= car.getLaneFinder().getPriorImage()(Rect(PLATECROP_X,PLATECROP_Y,PLATECROP_W,PLATECROP_H));
                cvtColor(plateCrop, grayImage1, COLOR_BGR2GRAY);
                cout<<"false"<<endl;
            }
            else
            {
                plateCrop= car.getLaneFinder().getCurrentImage()(Rect(PLATECROP_X,PLATECROP_Y,PLATECROP_W,PLATECROP_H));
                cvtColor(plateCrop, grayImage2, COLOR_BGR2GRAY);
                cout<<"true"<<endl;
                absdiff(grayImage1,grayImage2,differenceImage); 
                totalDiff = sum(differenceImage) / (grayImage1.cols * grayImage1.rows);
                cout << "sum diff: " <<totalDiff<<endl;
            }

            Mat lights;
            Mat bkPlateCrop;
            
            plateCrop.copyTo(bkPlateCrop); //HSV
            
            Mat temp = origin(Rect(PLATECROP_X,PLATECROP_Y,PLATECROP_W,PLATECROP_H));
            cvtColor(temp, temp, CV_RGB2GRAY);
            threshold(temp, temp, 60, 255, THRESH_BINARY_INV);
            medianBlur(temp, temp, 3);
            dilate(temp, temp, Mat(), Point(-1,-1));
            imshow("trhes",temp);
            
            cvtColor(plateCrop, lights, CV_BGR2HSV);
            inRange(lights,Scalar(LIGHTS_HUE_MIN, LIGHTS_SAT_MIN, LIGHTS_INT_MIN), Scalar(LIGHTS_HUE_MAX, LIGHTS_SAT_MAX, LIGHTS_INT_MAX),lights);
//            
            medianBlur(lights, lights, 3);
            dilate(lights, lights, Mat(), Point(-1,-1));

            
            car.getTrafficFinder().findCircles(lights);
            car.getTrafficFinder().drawCircles(plateCrop);
//
            car.getTrafficFinder().findSquares(bkPlateCrop);
//  //          car.getTrafficFinder().featureDetect(bkPlateCrop, car.getTrafficFinder().getSquares());
            car.getTrafficFinder().drawTraffic(plateCrop);
//            
           imshow("cur",plateCrop);
            
            car.getTrafficFinder().setSeqImage();
            
        }
 
        cvWaitKey(30);
    }

    return 0;
}