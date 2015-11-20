#include "MetaHeader.h"
#include "UV.h"
#include <future>

#define GAP 0
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
//bounding rectangle of the object, we will use the center of this as its position.
vector<Rect> objectBoundingRectangle;
//Rect objectBoundingRectangle2 = Rect(0,0,0,0); // 배열 처리

//int to string helper function
string intToString(int number){
    
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void searchForMovement(Mat thresholdImage, Mat &cameraFeed){
    
    bool objectDetected=false;
    Mat temp;
    thresholdImage.copyTo(temp);

    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours
    
    if(contours.size()>0)objectDetected=true;
    else objectDetected = false;
    
    if(objectDetected){
        //the largest contour is found at the end of the contours vector
        //we will simply assume that the biggest contour is the object we are looking for.
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
        
        circle(cameraFeed,Point(x,y),20,Scalar(0,255,0),2);
        line(cameraFeed,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
        line(cameraFeed,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
        line(cameraFeed,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
        line(cameraFeed,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
        putText(cameraFeed,"(" + intToString(x)+","+intToString(y)+")",Point(x,y),1,1,Scalar(255,0,0),2);
        
        ++itTheObject;
    }
}

void detectAndDisplay( Mat frame )
{
    std::vector<Rect> faces;
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
    //-- Show what you got
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
            
            bool debugMode = true;
            bool trackingEnabled = true;
    
            Mat frame1,frame2;
            Mat cropFrame1, cropFrame2;
            
            Mat oriConv = origin(Rect(10,340,700,200));
            
            Mat grayImage1,grayImage2;
            Mat differenceImage;
            Mat thresholdImage;
            Vec<double,4> totalDiff = 0.0;
            
            cropFrame1 = car.getLaneFinder().getPriorImage()(Rect(10,340,700,200));
            cvtColor(cropFrame1, grayImage1, COLOR_BGR2GRAY);
            
            cropFrame2 = car.getLaneFinder().getCurrentImage()(Rect(10,340,700,200));
            
            cvtColor(cropFrame2, grayImage2, COLOR_BGR2GRAY);
            absdiff(grayImage1,grayImage2,differenceImage);
            
            totalDiff = sum(differenceImage) / (690 * 140);
            cout << "sum diff: " <<totalDiff<<endl;
            
            if(totalDiff[0] > 14.0)
                continue;
            
            threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
            
            blur(thresholdImage, thresholdImage , cv::Size(BLUR_SIZE,BLUR_SIZE));

            cv::threshold(thresholdImage,thresholdImage,SENSITIVITY_VALUE,255,THRESH_BINARY);
            
            //if tracking enabled, search for contours in our thresholded image
            if(trackingEnabled){
                searchForMovement(thresholdImage, oriConv);
            }
            
            //show our captured frame
            imshow("moving",oriConv);
            
            //moving object
            //detect car in front of vehicle.
            Mat cropForVehicle;
            cropForVehicle = origin(Rect(LANECROP_X +50,LANECROP_Y - 50,LANECROP_W ,LANECROP_H/*- 50*/));
           // origin.copyTo(cropForVehicle);
            
            if( !face_cascade.load( face_cascade_name ) ){ printf("--(!)Error loading face cascade\n"); return -1; };
            if( !eyes_cascade.load( eyes_cascade_name ) ){ printf("--(!)Error loading eyes cascade\n"); return -1; };

            
                //-- 3. Apply the classifier to the frame
            detectAndDisplay( cropForVehicle );
            //imshow("gg",frame);
        }
        
 
        cvWaitKey(10);
    }

    return 0;
}