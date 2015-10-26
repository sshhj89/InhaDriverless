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
    
    VideoCapture video("/Users/sonhojun/Downloads/data/all.mp4");
    
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
//            imshow("bw",car.getLaneFinder().getbwYellowWhiteImage());
            
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
            imshow("line",laneCrop);
            
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
            
            Mat plateCrop = car.getLaneFinder().getCurrentImage()(Rect(PLATECROP_X,PLATECROP_Y,PLATECROP_W,PLATECROP_H));
            Mat lights;
            
//            Mat green;
//            cvtColor(plateCrop, green, CV_BGR2YCrCb);
//            
//            inRange(green,Scalar(0, 0, 0), Scalar(255, 120, 120),green);
//            
//            imshow("green",green);
            cvtColor(plateCrop, lights, CV_BGR2HSV);
            inRange(lights,Scalar(LIGHTS_HUE_MIN, LIGHTS_SAT_MIN, LIGHTS_INT_MIN), Scalar(LIGHTS_HUE_MAX, LIGHTS_SAT_MAX, LIGHTS_INT_MAX),lights);
            
            medianBlur(lights, lights, 3);
            dilate(lights, lights, Mat(), Point(-1,-1));
//            imshow("dilate lights",lights);
            
//            Mat squareMat;
//            cvtColor(plateCrop, squareMat, CV_BGR2HSV);
//            
//            inRange(squareMat,cv::Scalar(SQUARE_HUE_MIN, SQUARE_SAT_MIN, SQUARE_INT_MIN), Scalar(SQUARE_HUE_MAX, SQUARE_SAT_MAX, SQUARE_INT_MAX),squareMat);
//            
//            imshow("square range", squareMat);
            
            car.getTrafficFinder().findSquares(plateCrop);
            car.getTrafficFinder().debugSquares(car.getTrafficFinder().getSquares(), plateCrop);
            
            car.getTrafficFinder().findCircles(lights);
            car.getTrafficFinder().drawCircles(plateCrop);
            
            
            imshow("debug sq",plateCrop);
//            medianBlur(squareMat, squareMat, 3);
//            dilate(squareMat, squareMat, Mat(), Point(-1,-1)); // remove hole
            
 //           Canny(squareMat, squareMat, 100,350, 5);
//
//            Mat gray1;
//            cvtColor(plateCrop, gray1, CV_BGR2GRAY);
//            cv::Canny(gray1, // 그레이레벨 영상
//                      squareMat, // 결과 외곽선
//                      125,  // 낮은 경계값
//                      350);  // 높은 경계값
//            
//            // 넌제로 화소로 외곽선을 표현하므로 흑백 값을 반전
//            cv::Mat contoursInv; // 반전 영상
//            cv::threshold(squareMat, contoursInv, 128, 255, cv::THRESH_BINARY_INV);
//            // 밝기 값이 128보다 작으면 255가 되도록 설정
//            imshow("square range", contoursInv);
//            vector<Vec2f> lines;
//            HoughLines(squareMat, lines, 0.1, CV_PI/1800., 40);
//            vector<Vec2f>::const_iterator it = lines.begin();
//            while(it!=lines.end())
//            {
//                float rho = (*it)[0];
//                float theta = (*it)[1];
//                if(theta < CV_PI/4. || theta > 3.*CV_PI/4.)
//                {
//                    Point pt1(rho/cos(theta),0);
//                    Point pt2((rho-squareMat.rows*sin(theta))/cos(theta),squareMat.rows);
//                    line(plateCrop,pt1,pt2,Scalar(255,255,255),3);
//                }
//                else
//                {
//                    Point pt1(0,rho/sin(theta));
//                    Point pt2(squareMat.cols,(rho-squareMat.cols*sin(theta))/cos(theta));
//                    line(plateCrop,pt1,pt2,Scalar(0,0,0),3);
//                }
//                ++it;
//            }
//            imshow("line",plateCrop);
            
//            findContours(squareMat, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//            Mat drawing = Mat::zeros( squareMat.size(), CV_8UC3 );
//            for( int i = 0; i< contours.size(); i++ )
//            {
//                Scalar color = Scalar( i+150, i+150, i+150);
//                drawContours( drawing, contours, i, color);
//            }
//            imshow("contour",drawing);
//            
//            vector<Point> approx;
//            
//            for( size_t i = 0; i < contours.size(); i++ )
//            {
//                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.1, true);
//                
//                if( approx.size() == 4 &&
//                   fabs(contourArea(Mat(approx))) > 100 && fabs(contourArea(Mat(approx))) < 1000 &&
//                   isContourConvex(Mat(approx)) )
//                {
//                    double maxCosine = 0;
//                    
//                    for( int j = 2; j < 5; j++ )
//                    {
//                        // find the maximum cosine of the angle between joint edges
//                        double cosine = fabs(car.getTrafficFinder().angle(approx[j%4], approx[j-2], approx[j-1]));
//                        maxCosine = MAX(maxCosine, cosine);
//                    }
//                    
//                    if( maxCosine < 0.5 )
//                        car.getTrafficFinder().getSquares().push_back(approx);
//                }
//            }
//            
//            car.getTrafficFinder().drawSquares(plateCrop);
            
            
/*            newSquares.clear();
            heightwidth.clear();
            
            const vector<vector<Point>> tempSquare = car.getTrafficFinder().getSquares();
            const vector<Vec3f> tempCircle = car.getTrafficFinder().getCircles();
            
            for( size_t j = 0; j < tempSquare.size(); j++ )
            {
                int maxX = 0;
                int minX = 1000;
                int maxY = 0;
                int minY = 1000;
                
                for(int maxi = 0 ; maxi< 4; maxi++)
                {
                    if(maxX < tempSquare[j][maxi].x)
                    {
                        maxX = tempSquare[j][maxi].x;
                    }
                    
                    if(minX > tempSquare[j][maxi].x)
                    {
                        minX = tempSquare[j][maxi].x;
                    }
                    
                    if(maxY < tempSquare[j][maxi].y)
                    {
                        maxY = car.getTrafficFinder().getSquares()[j][maxi].y;
                    }
                    
                    if(minY > tempSquare[j][maxi].y)
                    {
                        minY = tempSquare[j][maxi].y;
                    }
                }
                heightwidth.push_back(Point(maxX - minX,maxY-minY));
            }
            
            //traffic lights 를 찾기 위함
            double trafficY = 0.;
            
            for( size_t i = 0; i < car.getTrafficFinder().getCircles().size(); i++ )
            {
                Point center(cvRound(tempCircle[i][0]), cvRound(tempCircle[i][1]));
                for( size_t j = 0; j < tempSquare.size(); j++ )
                {
                    int pX = cvRound((tempSquare[j][0].x + tempSquare[j][1].x + tempSquare[j][2].x + tempSquare[j][3].x) / 4);
                    int pY = cvRound((tempSquare[j][0].y + tempSquare[j][1].y + tempSquare[j][2].y + tempSquare[j][3].y) / 4);
                    
                    int maxX = 0;
                    int minX = 1000;
                    int maxY = 0;
                    int minY = 1000;
                    
                    for(int maxi = 0 ; maxi< 4; maxi++)
                    {
                        if(maxX < tempSquare[j][maxi].x)
                        {
                            maxX = tempSquare[j][maxi].x;
                        }
                        if(minX > tempSquare[j][maxi].x)
                        {
                            minX = tempSquare[j][maxi].x;
                        }
                        if(maxY < tempSquare[j][maxi].y)
                        {
                            maxY = tempSquare[j][maxi].y;
                        }
                        if(minY > tempSquare[j][maxi].y)
                        {
                            minY = tempSquare[j][maxi].y;
                        }
                    }
                    heightwidth.push_back(Point(maxX - minX,maxY-minY));
                    // 세로가 가로보다 길면 제거
                    if((maxY-minY) > (maxX - minX)* 1.2)
                        continue;
                    
                    //cout<<"p: "<<p.x<<" , "<<p.y<<endl;
                    if(abs(pX - center.x) < 20 && abs(pY - center.y) < 4)
                    {
                        newSquares.push_back(tempSquare.at(j));
                    }
                }
            }
            
            if(newSquares.size() > 0 )
            {
                car.getTrafficFinder().drawTraffic(plateCrop);
                cout<<"find traffic"<<endl;
                //Todo traffic point 계산
                
            }
            
            if(heightwidth.size() == 0)
                continue;
*/
            /**** detect unprotected left turn traffic signal *****/
/*            Mat sqaureCrop;
            int cols;
            int rows;
            
            cols = plateCrop.cols;
            rows = plateCrop.rows;
            
            for( size_t j = 0; j < tempSquare.size(); j++ )
            {
                
                Point p = tempSquare[j][0];
                
                int maxX = 0;
                int minX = 1000;
                int maxY = 0;
                int minY = 1000;
                
                for(int maxi = 0 ; maxi< 4; maxi++)
                {
                    if(maxX < tempSquare[j][maxi].x)
                    {
                        maxX = tempSquare[j][maxi].x;
                    }
                    if(minX > tempSquare[j][maxi].x)
                    {
                        minX = tempSquare[j][maxi].x;
                    }
                    if(maxY < tempSquare[j][maxi].y)
                    {
                        maxY = tempSquare[j][maxi].y;
                    }
                    if(minY > tempSquare[j][maxi].y)
                    {
                        minY = tempSquare[j][maxi].y;
                    }
                }
                
                cout<<"trafficY : " << trafficY << " , plate: " << (maxY - (maxY - minY)/2) << endl;
                //car.setState(0);
            }
 */
        }
 
        cvWaitKey(10);
    }

    return 0;
}