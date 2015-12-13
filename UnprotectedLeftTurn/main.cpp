#include "MetaHeader.h"
#include "UV.h"
#include "CTracker.h"
#include "kalman.h"

#include <future>

#define GAP 0
#define BOUNDARYDIFF 100
#define R 23



#define drawCross( center, color, d )                           \
line( img, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
line( img, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )

static double myX = cvRound((LANECROP_W + GAP)/2);
static double myY = cvRound(LANECROP_H + GAP /*-50*/);
static Point mousePos;

/** Global variables */
String face_cascade_name = "/Users/sonhojun/Downloads/opencv-3.0.0/data/haarcascades/haarcascade_cars1.xml";
String eyes_cascade_name = "/Users/sonhojun/Downloads/opencv-3.0.0/data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;


const static int SENSITIVITY_VALUE = 30;
const static int BLUR_SIZE = 15;
vector<int> theObject;

vector<Rect> objectBoundingRectangle;

#define decisionBound 2
int decisionArr[decisionBound] = {0,};
int idxFordecision = 0;
int memoryforState = 0;

#define BOUNDLIGHTS 2
int green[BOUNDLIGHTS] = {0,};
int idxLights = 0;
//Rect objectBoundingRectangle2 = Rect(0,0,0,0); // 배열 처리
Scalar Colors[]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,255,0),Scalar(0,255,255),Scalar(255,0,255),Scalar(255,127,255),Scalar(127,0,255),Scalar(127,0,127)};

//int to string helper function
string intToString(int number){
    
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == EVENT_MBUTTONDOWN )
    {
        cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if ( event == EVENT_MOUSEMOVE )
    {
        cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
        mousePos.x = x; mousePos.y= y;
    }
}

vector<Point> searchForMovement(Mat thresholdImage, Mat &cameraFeed){
    
    bool objectDetected=false;
    Mat temp;
    temp = thresholdImage.clone();

    vector< vector<Point> > contours;
    vector<Point> objectPoints;
    vector<Vec4i> hierarchy;
    findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours
    
    if(contours.size()>0)objectDetected=true;
    else objectDetected = false;
    
    if(objectDetected){

        vector<vector<Point>> largestContourVec;
        
        for(int i=0; i< contours.size(); i++)
            if(contourArea(contours[contours.size()-1-i]) > 20)
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
        
//        circle(cameraFeed,Point(x,y),10,Scalar(255,0,0),2);
//        putText(cameraFeed,"(" + intToString(x)+","+intToString(y)+")",Point(x,y),1,1,Scalar(0,255,0),2);
//        
        ++itTheObject;
    }
    
    return objectPoints;
}

bool boundaryCheck(Point prev, Point next)
{

    //Set manually ROI for left turn location: plate2.mp4
    Point p[4] = { Point(102,145), Point(307,122), Point(374,131), Point(130,166)};
    
    double MaxDist = 0.0;
    int projectionY = 0;
    
    for(int i=0; i < 4 ;i++)
    {

    }

    
    
    for(int i=0; i < 4 ;i++)
    {
        if(i == 3)
        {
          double tempDlast = pow((p[i].x - p[0].x),2)+pow((p[i].y - p[0].y),2);
          if(tempDlast > MaxDist)
                MaxDist = tempDlast;
            cout<<"Maxdist: " <<MaxDist<<endl;
            return false;
        }
        
        double tempD = pow((p[i].x - p[i+1].x),2)+pow((p[i].y - p[i+1].y),2);
        
        if(tempD > MaxDist)
            MaxDist = tempD;
        
    }
    cout<<MaxDist;
    
    return false;
}

vector<Point> detectAndDisplay( Mat &frame )
{
    vector<Rect> faces;
    Mat frame_gray;
    
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    vector<Point> centerP;
    
    //-- Detect faces
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
    
    for( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        if(center.x > frame.cols/2.) // remove cars on different lane
        {
            continue;
        }
        //ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
        circle(frame,center, 8, Scalar(0, 0, 255), -1, 1,0);
        centerP.push_back(center);
    }
    
    
    return centerP; // detected car center Point;
    
    //negative
}

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
    
    Mat origin1;
    Mat origin2;
    car.setState(LINEDETECTION);
    CTracker trackerFront(0.2,0.5,20.0);
    CTracker trackerLeft(0.2,0.5,60.0,10);
    
    namedWindow("My Window", 1);
    setMouseCallback("My Window", CallBackFunc, NULL);
    
    namedWindow("Left Window", 1);
    setMouseCallback("Left Window", CallBackFunc, NULL);

    bool canGo = false;
    while (1)
    {
        
        if(car.getState() == LINEDETECTION)
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
            //imshow("line",laneCrop);
            
            cout<<"currentLane : " << car.getOnLane() << endl;
            cout<<"possible line: " << car.getPossibleLine() << endl;
            cout<<"========================================="<<endl;
            laneCrop.release();
            temp.release();
        }
        
        if(car.getState() == STOPLINEDETECTED)
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
            //imshow("plates", canvas);
            
            /* Todo: make buffer for green lights.*/
            cout<<"green: "<<car.getTrafficFinder().getLightsColor() <<", unprotected: "<<car.getTrafficFinder().getUnprotected()<<endl;
            
            if(car.getTrafficFinder().getUnprotected() == 1)
            {
                car.setState(UNPROTECTEDDETECTED);
            }
        }
        
        bool first = true;
        
        if(car.getState() == UNPROTECTEDDETECTED)
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

            Mat frame1,frame2;
            Mat cropFrame1, cropFrame2;
            int D_W = DIFF_W;
            int D_H = DIFF_H;
            
            vector<Vec4i> kfhierarchy;
            vector<vector<Point> > kfcontours;
            vector<Point2f> kfcenters;
            
            vector<LineInfo> tInfo = car.getLaneFinder().getPositiveLines();

            if(car.getPossibleLine() == true)
            {
                                cout<<"slope: " <<tInfo[1].getSlope()<<endl;
                cout<<"interY: "<<tInfo[1].getIntX()<<", " << tInfo[1].getIntX()<<endl;
                D_W = tInfo[1].getPoint1X()+(LANECROP_X - DIFF_X) /*+ myX*/;
                D_H = tInfo[1].getPoint1Y()+(LANECROP_Y - DIFF_Y) - BOUNDARYDIFF;
            }
          
            cout<<"D_W: " << D_W<<", D_H: "<<D_H<<endl;
            Mat oriConv1 = origin(Rect(DIFF_X,DIFF_Y,D_W,D_H));  //for drawing
            Mat oriConv2 = origin(Rect(DIFF_X,DIFF_Y,D_W,D_H)); //for drawing
            
            //imshow("oriConv1 ", oriConv1);
            Mat arr = Mat::zeros(oriConv1.rows, oriConv1.cols, CV_8UC1);
            
            Mat grayImage1,grayImage2;
            Mat differenceImage1;
            Mat thresholdImage1;
            Vec<double,4> totalDiff = 0.0;
            
            cropFrame1 = origin1(Rect(DIFF_X,DIFF_Y,D_W,D_H));
            cvtColor(cropFrame1, grayImage1, COLOR_BGR2GRAY);
    
            cropFrame2 = origin2(Rect(DIFF_X,DIFF_Y,D_W,D_H));
    
            cvtColor(cropFrame2, grayImage2, COLOR_BGR2GRAY);
            absdiff(grayImage1,grayImage2,differenceImage1);
           
            totalDiff = sum(differenceImage1) / (cropFrame2.rows * cropFrame2.cols);
            
            cout << "sum diff: " <<totalDiff<<endl;
            
            if(totalDiff[0] > 6.0)
                continue;
            
            threshold(differenceImage1, thresholdImage1, SENSITIVITY_VALUE, 255, THRESH_BINARY);
            blur(thresholdImage1, thresholdImage1 , cv::Size(BLUR_SIZE,BLUR_SIZE));
            threshold(thresholdImage1,thresholdImage1,SENSITIVITY_VALUE,255,THRESH_BINARY);
            
            vector<Point> detectedMovingOb = searchForMovement(thresholdImage1, oriConv1);
            //imshow("thresh of moving on Left part ", thresholdImage1);
            
            //Set manually ROI for left turn location: plate2.mp4
            line(oriConv1,Point(102,145),Point(307,122),Scalar(0,0,255),2,CV_AA);
            line(oriConv1,Point(307,122),Point(374,131),Scalar(0,0,255),2,CV_AA);
            line(oriConv1,Point(374,131),Point(130,166),Scalar(0,0,255),2,CV_AA);
            line(oriConv1,Point(130,166),Point(102,145),Scalar(0,0,255),2,CV_AA);
            
            vector<Point>::const_iterator leftit = detectedMovingOb.begin();
            while(leftit != detectedMovingOb.end())
            {
                Point p = (*leftit);
                circle(oriConv1,p,10,Scalar(255,0,0),2);
                putText(oriConv1,"(" + intToString(p.x)+","+intToString(p.y)+")",p,1,1,Scalar(0,255,0),2);
                ++leftit;
            }
            
            
            vector<Vec4i> kfhierarchy4left;
            vector<vector<Point> > kfcontours4left;
            vector<Point2f> kfcenters4left;
            
            findContours(thresholdImage1, kfcontours4left, kfhierarchy4left, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
            
            vector<vector<Point> > contours_poly4left( kfcontours4left.size() );
            vector<Rect> boundRect4left( kfcontours4left.size() );
            
            Mat drawing1 = Mat::zeros(thresholdImage1.size(), CV_8UC1);
            for(size_t i = 0; i < kfcontours4left.size(); i++)
            {
                if(contourArea(kfcontours4left[i]) > 15)
                    drawContours(drawing1, kfcontours4left, i, Scalar::all(255), CV_FILLED, 8, vector<Vec4i>(), 0, Point());
            }
          
            
            for( size_t i = 0; i < kfcontours4left.size(); i++ )
            {
                approxPolyDP( Mat(kfcontours4left[i]), contours_poly4left[i], 3, true );
                boundRect4left[i] = boundingRect( Mat(contours_poly4left[i]) );
            }
            
            for( size_t i = 0; i < kfcontours4left.size(); i++ )
            {
                if(contourArea(kfcontours4left[i]) > 20)
                {
                    rectangle( drawing1, boundRect4left[i].tl(), boundRect4left[i].br(), Scalar(0, 255, 0), 2, 8, 0 );
                    Point center = Point(boundRect4left[i].x + (boundRect4left[i].width /2), boundRect4left[i].y + (boundRect4left[i].height/2));
                    circle(drawing1,center, 8, Scalar(0, 0, 255), -1, 1,0);
                    kfcenters4left.push_back(center);
                }
            }
            
            if(kfcenters4left.size()>0)
            {
                trackerLeft.Update(kfcenters4left);
                
                for(int i=0;i<trackerLeft.tracks.size();i++)
                {
                    if(trackerLeft.tracks[i]->trace.size()>1)
                    {
                        for(int j=0;j<trackerLeft.tracks[i]->trace.size()-1;j++)
                        {
                            line(oriConv1,trackerLeft.tracks[i]->trace[j],trackerLeft.tracks[i]->trace[j+1],Colors[trackerLeft.tracks[i]->track_id%9],2,CV_AA);
                        }
                        
                        cout<<"current" << trackerLeft.tracks[i]->trace[trackerLeft.tracks[i]->trace.size()-2]<<endl;
                        cout<<"predict" << trackerLeft.tracks[i]->trace[trackerLeft.tracks[i]->trace.size()-1]<<endl;
                        cout<<"=============="<<endl;
                        
                        boundaryCheck(trackerLeft.tracks[i]->trace[trackerLeft.tracks[i]->trace.size()-2], trackerLeft.tracks[i]->trace[trackerLeft.tracks[i]->trace.size()-1]);
                    }
                }
            }
            
            //imshow("Left Window", oriConv1);
            
            // front car
            Mat cropForVehicle1;
            Mat cropForVehicle2;
            Mat frontCarDrawing;
            cropForVehicle1 = origin1(Rect(LANECROP_X,LANECROP_Y - 50,LANECROP_W - 100,LANECROP_H/*- 50*/));
            cropForVehicle2 = origin2(Rect(LANECROP_X,LANECROP_Y - 50,LANECROP_W - 100,LANECROP_H/*- 50*/));
            frontCarDrawing = cropForVehicle2.clone();
            
            Mat arr1 = Mat::zeros(cropForVehicle2.rows, cropForVehicle2.cols, CV_8UC1);
            
            Mat grayImage3,grayImage4;
            Mat differenceImage2;
            Mat thresholdImage2;
            Vec<double,4> totalDiff1 = 0.0;
            
            cvtColor(cropForVehicle1, grayImage3, COLOR_RGB2GRAY);
            cvtColor(cropForVehicle2, grayImage4, COLOR_RGB2GRAY);
            absdiff(grayImage3,grayImage4,differenceImage2);
            
            totalDiff1 = sum(differenceImage2) / (cropForVehicle2.rows * cropForVehicle2.cols);
            
            cout << "sum diff: " <<totalDiff1<<endl;
            
            if(totalDiff1[0] > 6.0)
                continue;
            
            threshold(differenceImage2, thresholdImage2, SENSITIVITY_VALUE, 255, THRESH_BINARY);
            blur(thresholdImage2, thresholdImage2 , cv::Size(BLUR_SIZE,BLUR_SIZE));
//            medianBlur(thresholdImage2, thresholdImage2, 3);
            dilate(thresholdImage2, thresholdImage2, Mat(),Point(-1,-1));
            threshold(thresholdImage2,thresholdImage2,SENSITIVITY_VALUE,255,THRESH_BINARY);
            
            //imshow("thresh of frontcar ", thresholdImage2);
            
            vector<Vec4i> mvFronthierarchy;
            vector<vector<Point>> mvFrontcontours;
            vector<Point2f> mvFrontcenters;
            
            vector<Point> detectedMovingOb1 = searchForMovement(thresholdImage2, frontCarDrawing);
            circle(frontCarDrawing, Point(myX,myY-10), 3, Scalar(255,255,255));
           // imshow("moving on front", frontCarDrawing);
                
            if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading face cascade\n"); return -1; };
            if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading eyes cascade\n"); return -1; };
            

            findContours(thresholdImage2, kfcontours, kfhierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
            
            vector<vector<Point> > contours_poly( kfcontours.size() );
            vector<Rect> boundRect( kfcontours.size() );
            
            Mat drawing = Mat::zeros(thresholdImage2.size(), CV_8UC1);
            for(size_t i = 0; i < kfcontours.size(); i++)
            {
                if(contourArea(kfcontours[i]) > 30)
                    drawContours(drawing, kfcontours, i, Scalar::all(255), CV_FILLED, 8, vector<Vec4i>(), 0, Point());
            }
            
            Mat thresh_frame = drawing;
            //imshow("contour",thresh_frame);
            
            for( size_t i = 0; i < kfcontours.size(); i++ )
            {
                approxPolyDP( Mat(kfcontours[i]), contours_poly[i], 3, true );
                boundRect[i] = boundingRect( Mat(contours_poly[i]) );
            }
            cout<<"lane x1: "<<tInfo[1].getPoint1X()<<endl;
            cout<<"lane y1: "<<tInfo[1].getPoint1Y()<<endl;
            cout<<"lane x2: "<<tInfo[1].getPoint2X()<<endl;
            cout<<"lane y2: "<<tInfo[1].getPoint2Y()<<endl;
            cout<<"IntY : "<<tInfo[1].getIntY()<<endl;
            cout<<"IntX : "<<tInfo[1].getIntX()<<endl;
            cout<<"myX: "<< myX<<endl;
            cout<<"myY: " <<myY<<endl;
            cout<<"slope: " << tInfo[1].getSlope()<<endl;
            double reSlope = -tInfo[1].getSlope();
//            double reSlope = (double)(tInfo[1].getPoint2Y() - tInfo[1].getPoint1Y()) / (double)(tInfo[1].getPoint2X() - tInfo[1].getPoint1X());
            cout<<"reslope: " << reSlope<<endl;
            double reIntY = car.getLaneFinder().calInterceptY(tInfo[1].getPoint1Y(), reSlope, tInfo[1].getPoint1X());
            cout<<"reIntY: " << reIntY<<endl;
            double reIntX = car.getLaneFinder().calInterceptX(reIntY, reSlope);
            cout<<"reIntX: "<< reIntX<<endl;
            
            vector<Point>::const_iterator tit = detectedMovingOb1.begin();
            while(tit != detectedMovingOb1.end())
            {
                if(reSlope*((*tit).x) + reIntY < 0.)
                {
                    cout<<"erase"<<endl;
                    detectedMovingOb1.erase(tit);
                    continue;
                }
                
                circle(frontCarDrawing,Point((*tit).x,(*tit).y),10,Scalar(255,0,0),2);
                putText(frontCarDrawing,"(" + intToString((*tit).x)+","+intToString((*tit).y)+")",Point((*tit).x,(*tit).y),1,1,Scalar(0,255,0),2);

                ++tit;
            }

            
            line(frontCarDrawing,Point(tInfo[1].getPoint1X(),tInfo[1].getPoint1Y()+50),Point(reIntX,50 - 10),1,CV_AA);
            
            
            
            for( size_t i = 0; i < kfcontours.size(); i++ )
            {
                if(contourArea(kfcontours[i]) > 20)
                {
                    rectangle( thresh_frame, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 2, 8, 0 );
                    Point center = Point(boundRect[i].x + (boundRect[i].width /2), boundRect[i].y + (boundRect[i].height/2));
                    circle(thresh_frame,center, 8, Scalar(0, 0, 255), -1, 1,0);
                    cout<<"kalman center: "<< center<<endl;
                    cout<<"center x : " << (LANECROP_W - 100)/2<<endl;
                    if(reSlope*center.x + reIntY >= 0.)
                        kfcenters.push_back(center);
                }
            }
            
            if(kfcenters.size()>0)
            {
                trackerFront.Update(kfcenters);
        
                for(int i=0;i<trackerFront.tracks.size();i++)
                {
                    if(trackerFront.tracks[i]->trace.size()>1)
                    {
                        for(int j=0;j<trackerFront.tracks[i]->trace.size()-1;j++)
                        {
                            line(frontCarDrawing,trackerFront.tracks[i]->trace[j],trackerFront.tracks[i]->trace[j+1],Colors[trackerFront.tracks[i]->track_id%9],2,CV_AA);
                            
                        }
                        
                        cout<<"current" << trackerFront.tracks[i]->trace[trackerFront.tracks[i]->trace.size()-2]<<endl;
                        cout<<"predict" << trackerFront.tracks[i]->trace[trackerFront.tracks[i]->trace.size()-1]<<endl;
                        cout<<"=============="<<endl;
                        
                        boundaryCheck(trackerFront.tracks[i]->trace[trackerFront.tracks[i]->trace.size()-2], trackerFront.tracks[i]->trace[trackerFront.tracks[i]->trace.size()-1]);
                    }
                }
            }
           // imshow("haar+kf+moving", frontCarDrawing);
            vector<Point> frontCarCenter = detectAndDisplay(frontCarDrawing);
           // imshow("moving on front of Haar", frontCarDrawing);
            cout<<"========================"<<endl;
            
            if(frontCarCenter.size() == 0)
            {
                cout<<"num of frontcar: " <<frontCarCenter.size()<<endl;
                canGo = true;
                if(kfcenters.size()>0)
                {
                    for(int i=0;i<trackerFront.tracks.size();i++)
                    {
                        if(trackerFront.tracks[i]->trace.size()>1)
                        {
                            for(int j=0;j<trackerFront.tracks[i]->trace.size()-1;j++)
                            {
                                Point c =trackerFront.tracks[i]->trace[trackerFront.tracks[i]->trace.size()-j-2];
                                Point p = trackerFront.tracks[i]->trace[trackerFront.tracks[i]->trace.size()-j-1];
                                vector<Point>::const_iterator it = detectedMovingOb1.begin();
                                while(it != detectedMovingOb1.end())
                                {
                                    double distp = sqrt(pow((p.x - (*it).x),2) + pow((p.y - (*it).y),2));
                                    double distc = sqrt(pow((c.x - (*it).x),2) + pow((c.y - (*it).y),2));
                                    
                                    cout<<"distp: " <<distp<<", "<<distc<<endl;
                                    if(distp < 6. || distc < 6.)
                                    {   canGo = false;
                                        break;
                                    }
                                    ++it;
                                }
                                
                                if(canGo == false)
                                    break;

                            }
                        }
                        if(canGo == false)
                            break;
                        
                    }
                }
            }
            
            vector<Point>::const_iterator itCarCenter =frontCarCenter.begin(); //result of Haar.
            vector<Point>::const_iterator mvIt = detectedMovingOb1.begin();
            
            while(itCarCenter != frontCarCenter.end())
            {
                Point p = (*itCarCenter);
                 cout<<"p: " << p<<endl;
                while(mvIt != detectedMovingOb1.end())
                {
                    double dist = sqrt(pow((p.x - (*mvIt).x),2) + pow((p.y - (*mvIt).y),2));
                   
                    cout<<"mov: " << (*mvIt)<<endl;
                    cout<<"dist: " << dist<<endl;
                    if(dist < 16.)  // fit to image
                    {
                        cout<<"dist Min: " << dist<<endl;
                        canGo = false;
                        break;
                    }
                    else
                    {
                        cout<<"here1"<<endl;
                        canGo = true;
                    }
            
                    ++mvIt;
                }
            
                if(canGo == false)
                {
                    cout<<"here2"<<endl;
                    break;
                }
                itCarCenter++;
            
                }
                        
                if(canGo == true)
                {
                    decisionArr[idxFordecision++] = 1;
                    putText(frontCarDrawing,"CAN GO!!!!",Point(100,100),1,1,Scalar(255,255,255),2);
                }else
                {
                    decisionArr[idxFordecision++] = -1;
                    putText(frontCarDrawing,"CANNOT GO!!!!",Point(100,100),1,1,Scalar(255,255,255),2);
                }
            
            
            
           //trafficlights
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
            
            if(car.getTrafficFinder().getLightsColor() == 1)
            {
                green[idxLights++] = 1;
            }
            if(idxLights == BOUNDLIGHTS)
            {
                idxLights = 0;
            }

            //imshow("lights", canvas);
            
            
            if(idxFordecision == decisionBound)
            {
                int sum = 0;
                
                for(int i=0; i<decisionBound; i++)
                    sum += decisionArr[i];
                
                int lightsCheck = 0;
                for(int j=0; j< BOUNDLIGHTS; j++)
                {
                    lightsCheck += green[j];
                }
                idxFordecision = 0;
                
            }
            
            imshow("My Window",frontCarDrawing);
            cvWaitKey(0);
           // system("afplay /Users/sonhojun/Downloads/beep-05.mp3&");
        }
        cvWaitKey(10);

        
    }

    return 0;
}