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
       //     imshow("line",laneCrop);
            
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
                plateCrop= origin(Rect(PLATECROP_X,PLATECROP_Y,PLATECROP_W,PLATECROP_H));
                cvtColor(plateCrop, grayImage1, CV_BGR2GRAY);
                cout<<"false"<<endl;
            }
            else
            {
                plateCrop= origin(Rect(PLATECROP_X,PLATECROP_Y,PLATECROP_W,PLATECROP_H));
                cvtColor(plateCrop, grayImage2, CV_BGR2GRAY);
                cout<<"true"<<endl;
                absdiff(grayImage1,grayImage2,differenceImage); 
                totalDiff = sum(differenceImage) / (grayImage1.cols * grayImage1.rows);
                cout << "sum diff: " <<totalDiff<<endl;
            }

            Mat redlights;
            Mat greenlights;
            Mat greenHsv;
            Mat bkPlateCrop;
            Mat canvas;
            plateCrop.copyTo(canvas); //RGB
            
            plateCrop.copyTo(bkPlateCrop); //RGB

            Mat temp = origin(Rect(PLATECROP_X,PLATECROP_Y,PLATECROP_W,PLATECROP_H));
            car.getTrafficFinder().setOrigin(temp);
            
//            cvtColor(temp, temp, CV_RGB2GRAY);
//            threshold(temp, temp, 60, 255, THRESH_BINARY_INV);
//           // medianBlur(temp, temp, 3);
//            dilate(temp, temp, Mat(), Point(-1,-1));
//            imshow("trhes",temp);
            
            cvtColor(plateCrop, redlights, CV_BGR2HSV);
             cvtColor(redlights, redlights, CV_BGR2HSV);
           // imshow("HSV",redlights);
            inRange(redlights,Scalar(RED_HUE_MIN, RED_SAT_MIN, RED_INT_MIN), Scalar(RED_HUE_MAX, RED_SAT_MAX, RED_INT_MAX),redlights);
//            
           //medianBlur(lights, lights, 5);
            dilate(redlights, redlights, Mat(), Point(-1,-1));
            dilate(redlights,redlights,Mat(),Point(1,1));
            imshow("red",redlights);
//            
            car.getTrafficFinder().findCircles(redlights);
            car.getTrafficFinder().drawCircles(canvas);
//
            cvtColor(plateCrop, greenHsv, CV_BGR2HSV);
           // imshow("HSV1",greenHsv);
            cvtColor(greenHsv, greenHsv, CV_BGR2HSV);
           // imshow("HSV2",greenHsv);
            inRange(greenHsv,Scalar(LIGHTS_HUE_MIN, LIGHTS_SAT_MIN, LIGHTS_INT_MIN), Scalar(LIGHTS_HUE_MAX, LIGHTS_SAT_MAX, LIGHTS_INT_MAX),greenHsv);
            //
            medianBlur(greenHsv, greenHsv, 5);
            dilate(greenHsv, greenHsv, Mat(), Point(-1,-1));
            dilate(greenHsv,greenHsv,Mat(),Point(1,1));
           //imshow("greenHSV",greenHsv);

    
            Mat image_r( plateCrop.rows, plateCrop.cols, CV_8UC1);
            Mat image_g( plateCrop.rows, plateCrop.cols, CV_8UC1);
            Mat image_b( plateCrop.rows, plateCrop.cols, CV_8UC1);
            
            Mat out[] = { image_b, image_g, image_r };
            int from_to[] = { 0,0, 1,1, 2,2 };
            mixChannels( &plateCrop, 1, out, 3, from_to, 3 );
        
//            imshow( "red di", image_r );
//            imshow( "green di", image_g );
//            imshow( "blue di", image_b );
//            
            double minVal; double maxVal; Point minLoc; Point maxLoc;
            Point matchLoc;
            
            minMaxLoc( image_g, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
            
            cout<<"maxVal: "<<maxVal<<"maxLoc: "<<maxLoc<<endl;
            double avgGreen = (sum(image_g) / (image_g.cols * image_g.rows))[0];
            Scalar     mean;
            Scalar     stddev;
            
            cv::meanStdDev ( image_g, mean, stddev );
            int       mean_pxl = mean.val[0];
            int     stddev_pxl = stddev.val[0];
            
           // equalizeHist(image_g, image_g);
            int dev = sqrt(stddev_pxl);
            
//            cout<<"avg: "<<avgGreen<<", "<<mean_pxl<<", "<<"dev: "<<stddev_pxl<<endl;
            threshold(image_g, image_g, mean_pxl+2*dev, 255, THRESH_BINARY);
            bitwise_and(greenHsv, image_g, image_g);
//            Point center(maxLoc.x), cvRound(maxLoc.y);
//            int radius = 3;
//            circle( image_g, center, 3, Scalar(255,255,255), -1, 8, 0 );
//            circle( image_g, center, radius, Scalar(255,255,255), 3, 8, 0 );
            
            medianBlur(greenlights, greenlights, 3);
            dilate(greenlights, greenlights, Mat(), Point(-1,-1));
            imshow( "green", image_g );
            dilate(image_g, image_g, Mat(), Point(-1,-1));
           // dilate(greenHsv,greenHsv,Mat(),Point(1,1));
            
            car.getTrafficFinder().findCircles(image_g);
            car.getTrafficFinder().drawCircles(canvas);
//
            
            //cvtColor(plateCrop, greenlights, CV_BGR2HSV);
            //imshow("HSV",greenlights);
//            inRange(greenlights,Scalar(LIGHTS_HUE_MIN, LIGHTS_SAT_MIN, LIGHTS_INT_MIN), Scalar(LIGHTS_HUE_MAX, LIGHTS_SAT_MAX, LIGHTS_INT_MAX),greenlights);
//            //
//            //medianBlur(lights, lights, 5);
//            dilate(greenlights, greenlights, Mat(), Point(-1,-1));
//            dilate(greenlights,greenlights,Mat(),Point(1,1));
//            imshow("green",greenlights);
            
//
//            
//            Mat greenCir;
//            int N=5;
//            for( int l = 0; l < N; l++ )
//            {
//                
//                
//                    greenCir = image_g >= (l+1)*255/N;
//                    // bitwise_not(gray, lights, gray);
//                    if(l == 1)
//                        imshow("l1",greenCir);
//                    else if (l ==2)
//                    {    imshow("l2",greenCir); break;}
//                    else if(l==3)
//                        imshow("l3",greenCir);
//                    else if(l==4)
//                        imshow("l4",greenCir);
//                    else
//                        imshow("l5",greenCir);
//                    
//                
//                
//            }
//
//            
//            car.getTrafficFinder().findCircles(greenCir);
//            car.getTrafficFinder().drawCircles(canvas);
//            
//
////
                      
            cvtColor(plateCrop, bkPlateCrop, CV_BGR2HSV);
            car.getTrafficFinder().findSquares(bkPlateCrop);
           // cvWaitKey(1000);
            car.getTrafficFinder().drawSquares(canvas);
            imshow("circles", canvas);
            
//            car.getTrafficFinder().featureDetect(bkPlateCrop, car.getTrafficFinder().getSquares());
//            car.getTrafficFinder().drawTraffic(plateCrop);
//            
//           imshow("cur",plateCrop);
            
            car.getTrafficFinder().setSeqImage();
            
        }
 
        cvWaitKey(30);
    }

    return 0;
}