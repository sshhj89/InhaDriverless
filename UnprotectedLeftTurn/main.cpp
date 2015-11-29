#include "MetaHeader.h"
#include "UV.h"
#include "CTracker.h"
#include "kalman.h"

#include <future>

#define GAP 0
#define BOUNDARYDIFF 50
#define R 23
using namespace std;

static double myX = cvRound((LANECROP_W + GAP)/2);
static double myY = cvRound(LANECROP_H + GAP /*-50*/);


/** Global variables */
String face_cascade_name = "/Users/sonhojun/Downloads/opencv-3.0.0/data/haarcascades/haarcascade_cars1.xml";
String eyes_cascade_name = "/Users/sonhojun/Downloads/opencv-3.0.0/data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;


const static int SENSITIVITY_VALUE = 30;
const static int BLUR_SIZE = 15;
vector<int> theObject;

vector<Rect> objectBoundingRectangle;
//Rect objectBoundingRectangle2 = Rect(0,0,0,0); // 배열 처리
Scalar Colors[]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,255,0),Scalar(0,255,255),Scalar(255,0,255),Scalar(255,127,255),Scalar(127,0,255),Scalar(127,0,127)};

//int to string helper function
string intToString(int number){
    
    std::stringstream ss;
    ss << number;
    return ss.str();
}
void calObject(vector<Point> frame1Ob, vector<Point> frame2Ob, Mat &image, Mat& mArr)
{
    vector<Point>::const_iterator itFrame1Ob1 = frame1Ob.begin();
    int x1;
    int y1;
    while(itFrame1Ob1 != frame1Ob.end())
    {
        x1 = (*itFrame1Ob1).x;
        y1 = (*itFrame1Ob1).y;
        mArr.at<uchar>(y1,x1) = 1;
        ++itFrame1Ob1;
    }
    
//    vector<Point>::const_iterator itFrame1Ob1 = frame1Ob.begin();
//    vector<Point>::const_iterator itFrame1Ob2 = frame1Ob.begin();
//    
//    vector<Point> newFrame10b2;
//    int x1 = 0;
//    int y1 = 0;
//    int x2 = 0;
//    int y2 = 0;
//    while(itFrame1Ob1 != frame1Ob.end())
//    {
//        x1 = (*itFrame1Ob1).x;
//        y1 = (*itFrame1Ob1).y;
//        while(itFrame1Ob2 != frame1Ob.end())
//        {
//            x2 = (*itFrame1Ob2).x;
//            y2 = (*itFrame1Ob2).y;
//            if(x1 == x2 && y1 == y2)
//            {
//                ++itFrame1Ob2;
//                continue;
//            }
//            else
//            {
//                double distance = pow((x1-x2),2) + pow((y1-y2),2);
//                cout<<"distatnce: " <<distance<<endl;
//                if( R*R > distance)
//                {
//                    //묶어 주자
//                    
//                    newFrame10b2.push_back(Point((x1+x2)/2,(y1+y2)/2));
//                }
//                else
//                {
//                     newFrame10b2.push_back(Point(x1,y1));
//                }
//            }
//            ++itFrame1Ob2;
//        }
//        ++itFrame1Ob1;
//    }
//    
//    vector<Point>::const_iterator itTheObject = newFrame10b2.begin();
//
//    while(itTheObject != newFrame10b2.end())
//    {
//        cout<<"here"<<endl;
//        circle(image,Point((*itTheObject).x,(*itTheObject).y),10,Scalar(0,0,255),2);
//        putText(image,"(" + intToString((*itTheObject).x)+","+intToString((*itTheObject).y)+")",Point((*itTheObject).x,(*itTheObject).y),1,1,Scalar(0,0,255),2);
//        
//        ++itTheObject;
//    }
//
}

vector<Point> searchForMovement(Mat thresholdImage, Mat &cameraFeed){
    
    bool objectDetected=false;
    Mat temp;
    thresholdImage.copyTo(temp);

    vector< vector<Point> > contours;
    vector<Point> objectPoints;
    vector<Vec4i> hierarchy;
    findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours
    
    if(contours.size()>0)objectDetected=true;
    else objectDetected = false;
    
    if(objectDetected){

        vector<vector<Point>> largestContourVec;
        
        for(int i=0; i< contours.size(); i++)
            largestContourVec.push_back(contours.at(contours.size()-1-i));
        
        vector<vector<Point>>::const_iterator itLargestConture = largestContourVec.begin();
        
        theObject.clear();
        int xpos = 0;
        int ypos = 0;
        while(itLargestConture != largestContourVec.end())
        {
            objectBoundingRectangle.push_back(boundingRect((*itLargestConture)));
            xpos = boundingRect((*itLargestConture)).x+boundingRect((*itLargestConture)).width/2;
            ypos = boundingRect((*itLargestConture)).y+boundingRect((*itLargestConture)).height/2;
            
            theObject.push_back(xpos);
            theObject.push_back(ypos);
            objectPoints.push_back(Point(xpos,ypos));
            ++itLargestConture;
        }
        
        
    }
    
    vector<int>::const_iterator itTheObject = theObject.begin();
    bool secondWhile = false;
    int x = 0;
    int y = 0;
    while(itTheObject != theObject.end())
    {
        if(secondWhile == false)
        {
            x= (*itTheObject);
            secondWhile = true;
            ++itTheObject;
            continue;
        }
        else
        {
            y = (*itTheObject);
            secondWhile = false;
        }
        
        circle(cameraFeed,Point(x,y),10,Scalar(255,0,0),2);
        putText(cameraFeed,"(" + intToString(x)+","+intToString(y)+")",Point(x,y),1,1,Scalar(0,255,0),2);
        
        ++itTheObject;
    }
    
    return objectPoints;
}

void detectAndDisplay( Mat frame )
{
    vector<Rect> faces;
    Mat frame_gray;
    
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    
    //-- Detect faces
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
    
    for( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        if(center.x > frame.cols/2.) // remove cars on different lane
        {
            continue;
        }
        ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }

    imshow("infront car", frame );
}

void putTextonImage(string t, const Mat& image)
{
    string text = "current state: " + t;
    putText(image, text, Point(10, 130),1, 1.0, cvScalar(255, 255, 255, 0));
}

int main()
{
    //ocl::setUseOpenCL(true);
    
    VideoCapture video("/Users/sonhojun/Downloads/data/plate2.mp4");
    
    if(!video.isOpened())
    {
        cout<<"video open error"<<endl;
        return 0 ;
    }
    
    Mat temp;
    Mat origin;
    UVCar car = UVCar();
    int chLane;
    
    Mat origin1;
    Mat origin2;
    car.setState(LINEDETECTION);
    CTracker tracker(0.2,0.5,60.0,10,20);
    while (1)
    {
        video.read(temp);
        origin1 = temp.clone();
        if(temp.channels() == 3)
            cvtColor(temp, temp, CV_BGR2HSV);
        car.getLaneFinder().setPriorImage(temp);
        
        video.read(temp);
        temp.copyTo(origin);
        origin2 = temp.clone();
        
        if(temp.channels() == 3)
            cvtColor(temp, temp, CV_BGR2HSV);
        
        car.getLaneFinder().setCurrentImage(temp);
        
        if(car.getState() == LINEDETECTION)
        {
            Mat laneCrop = car.getLaneFinder().getPriorImage()(Rect(LANECROP_X - GAP,LANECROP_Y + GAP,LANECROP_W + GAP,LANECROP_H + GAP /*- 50*/));
            
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
            imshow("line",laneCrop);
            
            cout<<"currentLane : " << car.getOnLane() << endl;
            cout<<"possible line: " << car.getPossibleLine() << endl;
            cout<<"========================================="<<endl;
            laneCrop.release();
            temp.release();
        }
        
        if(car.getState() == STOPLINEDETECTED)
        {
            Mat plateCrop;
            Mat greenlights;
            Mat bkPlateCrop;
            Mat canvas;
            
            plateCrop= origin(Rect(PLATECROP_X,PLATECROP_Y,PLATECROP_W,PLATECROP_H));
            car.getTrafficFinder().setOrigin(plateCrop);
            
            plateCrop.copyTo(canvas); //RGB
            plateCrop.copyTo(bkPlateCrop); //RGB
            
            car.getTrafficFinder().setGreenHSV(plateCrop);
            if(car.getTrafficFinder().getGreenHsv().size == 0)
            {
                cout<<"no greenHsv Mat"<<endl;
                continue;
            }
            car.getTrafficFinder().splitAsGreenBin(plateCrop);
           
            bitwise_and(car.getTrafficFinder().getGreenHsv(), car.getTrafficFinder().getGreenBin(), car.getTrafficFinder().getGreenBin());
            dilate(car.getTrafficFinder().getGreenBin(), car.getTrafficFinder().getGreenBin(), Mat(), Point(-1,-1));
            
            car.getTrafficFinder().findCircles(car.getTrafficFinder().getGreenBin());
            car.getTrafficFinder().drawCircles(canvas);
            
            cvtColor(plateCrop, bkPlateCrop, CV_BGR2HSV);
            car.getTrafficFinder().findSquares(bkPlateCrop);
            car.getTrafficFinder().drawSquares(canvas);
            imshow("circles", canvas);
            
            /* Todo: make buffer for green lights.*/
            cout<<"green: "<<car.getTrafficFinder().getLightsColor() <<", unprotected: "<<car.getTrafficFinder().getUnprotected()<<endl;
            if(car.getTrafficFinder().getUnprotected() == 1)
            {
                car.setState(UNPROTECTEDDETECTED);
            }
        }
        
        if(car.getState() == UNPROTECTEDDETECTED)
        {
            
            Mat frame1,frame2;
            Mat cropFrame1, cropFrame2;
            int D_W = DIFF_W;
            int D_H = DIFF_H;
            
            
            vector<Vec4i> kfhierarchy;
            vector<vector<Point> > kfcontours;
            vector<Point2f> kfcenters;
            

            if(car.getPossibleLine() == true)
            {
                vector<LineInfo> tInfo = car.getLaneFinder().getPositiveLines();
                D_W = tInfo[1].getPoint1X()+(LANECROP_X - DIFF_X) + myX;
                D_H = tInfo[1].getPoint1Y()+(LANECROP_Y - DIFF_Y) - BOUNDARYDIFF;
            }
          
            cout<<"D_W: " << D_W<<", D_H: "<<D_H<<endl;
            Mat oriConv = origin(Rect(DIFF_X,DIFF_Y,D_W,D_H));
            Mat oriConv2 = origin(Rect(DIFF_X,DIFF_Y,D_W,D_H));
            
            Mat arr = Mat::zeros(oriConv.rows, oriConv.cols, CV_8UC1);
            
            Mat grayImage1,grayImage2;
            Mat differenceImage1;
            Mat thresholdImage1;
            Vec<double,4> totalDiff = 0.0;
            
            cropFrame1 = origin1(Rect(DIFF_X,DIFF_Y,D_W,D_H));
            cvtColor(cropFrame1, grayImage1, COLOR_RGB2GRAY);
    
            cropFrame2 = origin2(Rect(DIFF_X,DIFF_Y,D_W,D_H));
    
            cvtColor(cropFrame2, grayImage2, COLOR_RGB2GRAY);
            absdiff(grayImage1,grayImage2,differenceImage1);
           
            totalDiff = sum(differenceImage1) / (cropFrame2.rows * cropFrame2.cols);
            
            cout << "sum diff: " <<totalDiff<<endl;
            
            if(totalDiff[0] > 6.0)
                continue;
            
            threshold(differenceImage1, thresholdImage1, SENSITIVITY_VALUE, 255, THRESH_BINARY);
            blur(thresholdImage1, thresholdImage1 , cv::Size(BLUR_SIZE,BLUR_SIZE));
            threshold(thresholdImage1,thresholdImage1,SENSITIVITY_VALUE,255,THRESH_BINARY);
//            imshow("threshhold1",thresholdImage1);
            //vector<Point> frame1Objects = searchForMovement(thresholdImage1, oriConv);
            //calObject(frame1Objects, frame1Objects, oriConv, arr);
            //imshow("moving1",oriConv);
          
            findContours(thresholdImage1, kfcontours, kfhierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
            
            vector<vector<Point> > contours_poly( kfcontours.size() );
            vector<Rect> boundRect( kfcontours.size() );
            
            Mat drawing = Mat::zeros(oriConv.size(), CV_8UC1);
            for(size_t i = 0; i < kfcontours.size(); i++)
            {
                //          cout << contourArea(contours[i]) << endl;
                if(contourArea(kfcontours[i]) > 200)
                    drawContours(drawing, kfcontours, i, Scalar::all(255), CV_FILLED, 8, vector<Vec4i>(), 0, Point());
            }
            
            Mat thresh_frame = drawing;
            imshow("contour",thresh_frame);
            
            for( size_t i = 0; i < kfcontours.size(); i++ )
                        {
                            approxPolyDP( Mat(kfcontours[i]), contours_poly[i], 3, true );
                            boundRect[i] = boundingRect( Mat(contours_poly[i]) );
                        }
                
                        for( size_t i = 0; i < kfcontours.size(); i++ )
                        {
                            if(contourArea(kfcontours[i]) > 1000){
                                rectangle( thresh_frame, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 2, 8, 0 );
                                // Rect r = boundRect[i];
                                //  frame(r).copyTo(images_array);
                                // img_array.push_back(images_array);
                                Point center = Point(boundRect[i].x + (boundRect[i].width /2), boundRect[i].y + (boundRect[i].height/2));
                                circle(thresh_frame,center, 8, Scalar(0, 0, 255), -1, 1,0);
                                kfcenters.push_back(center);
                            }
                        }
                
                
                        if(kfcenters.size()>0)
                        {
                            tracker.Update(kfcenters);
                
                            cout << tracker.tracks.size()  << endl;
                
                            for(int i=0;i<tracker.tracks.size();i++)
                            {
                                if(tracker.tracks[i]->trace.size()>1)
                                {
                                    for(int j=0;j<tracker.tracks[i]->trace.size()-1;j++)
                                    {
                                        line(oriConv,tracker.tracks[i]->trace[j],tracker.tracks[i]->trace[j+1],Colors[tracker.tracks[i]->track_id%9],2,CV_AA);
                                        //drawCross(frame, tracker.tracks[i]->trace[j], Scalar(255, 255, 255), 5);
                                    }
                                }
                            }
                        }
                        
                        
                        imshow("OUTPUT Fist Camera",oriConv);
            
            
//            Mat grayImage3,grayImage4;
//            Mat differenceImage2;
//            Mat thresholdImage2;
//            
//            Mat frame3, frame4;
//            video.read(frame3);
//            video.read(frame3);
//            
//            video.read(frame3);
//            Mat cropFrame3 = frame3(Rect(DIFF_X,DIFF_Y,D_W,D_H));
//            cvtColor(cropFrame3, grayImage3, COLOR_RGB2GRAY);
//
//            video.read(frame4);
//            Mat cropFrame4 = frame4(Rect(DIFF_X,DIFF_Y,D_W,D_H));
//            cvtColor(cropFrame4, grayImage4, COLOR_RGB2GRAY);
//
//            absdiff(grayImage3,grayImage4,differenceImage2);
//            
////            imshow("differ2",differenceImage2);
//            threshold(differenceImage2, thresholdImage2, SENSITIVITY_VALUE, 255, THRESH_BINARY);
//            blur(thresholdImage2, thresholdImage2 , cv::Size(BLUR_SIZE,BLUR_SIZE));
//            threshold(thresholdImage2,thresholdImage2,SENSITIVITY_VALUE,255,THRESH_BINARY);
// //           imshow("threshhold2",thresholdImage2);
//            vector<Point> frame2Objects = searchForMovement(thresholdImage2, oriConv2);
//            imshow("moving2",oriConv2);
            
            
            Mat cropForVehicle;
            cropForVehicle = origin(Rect(LANECROP_X +50,LANECROP_Y - 50,LANECROP_W ,LANECROP_H/*- 50*/));
            
            if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading face cascade\n"); return -1; };
            if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading eyes cascade\n"); return -1; };

            detectAndDisplay( cropForVehicle );
           // cvWaitKey(1000);
           // cvWaitKey(0);
        }
        
 
        cvWaitKey(10);
    }

    return 0;
}