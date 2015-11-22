//
//  MovingFinder.cpp
//  UnprotectedLeftTurn
//
//  Created by SonHojun on 10/9/15.
//  Copyright (c) 2015 SonHojun. All rights reserved.
//

#include "MovingFinder.h"

//our sensitivity value to be used in the threshold() function
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
    //these two vectors needed for output of findContours
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    //findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
    findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours
    
    //if contours vector is not empty, we have found some objects
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

int main(){
    
    bool objectDetected = true;
    bool debugMode = true;
    bool trackingEnabled = true;

    bool pause = false;

    Mat frame1,frame2;
    Mat cropFrame1, cropFrame2;
    
    Mat grayImage1,grayImage2;
    Mat differenceImage;
    Mat thresholdImage;
    Vec<double,4> totalDiff = 0.0;
    
    VideoCapture capture;
    
    while(1){
        
        //we can loop the video by re-opening the capture every time the video reaches its last frame
        
        
        capture.open("/Users/sonhojun/Downloads/data/plate2.mp4");
        
        if(!capture.isOpened()){
            cout<<"ERROR ACQUIRING VIDEO FEED\n";
            getchar();
            return -1;
        }
        
        //check if the video has reach its last frame.
        //we add '-1' because we are reading two frames from the video at a time.
        //if this is not included, we get a memory error!
        while(capture.get(CV_CAP_PROP_POS_FRAMES)<capture.get(CV_CAP_PROP_FRAME_COUNT)-1){
            
            capture.read(frame1);
            cropFrame1 = frame1(Rect(10,340,700,200));
            cvtColor(cropFrame1, grayImage1, COLOR_BGR2GRAY);
            
            capture.read(frame2);
            cropFrame2 = frame2(Rect(10,340,700,200));
            
            cvtColor(cropFrame2, grayImage2, COLOR_BGR2GRAY);
            absdiff(grayImage1,grayImage2,differenceImage);
            
            totalDiff = sum(differenceImage) / (690 * 140);
            cout << "sum diff: " <<totalDiff<<endl;
            
            if(totalDiff[0] > 14.0)
                continue;
            
            threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
            if(debugMode==true){
                //show the difference image and threshold image
                imshow("Difference Image", differenceImage);
                imshow("Threshhold Image", thresholdImage);
            }else{
                //if not in debug mode, destroy the windows so we don't see them anymore
                destroyWindow("Difference Image");
                destroyWindow("Threshhold Image");
            }

            blur(thresholdImage, thresholdImage , cv::Size(BLUR_SIZE,BLUR_SIZE));
            //GaussianBlur(thresholdImage, thresholdImage, Size(BLUR_SIZE,BLUR_SIZE), 0,0);
            //increase the size of the object we are trying to track. (Much like dilate and erode)
            
            //threshold again to obtain binary image from blur output
            cv::threshold(thresholdImage,thresholdImage,SENSITIVITY_VALUE,255,THRESH_BINARY);
            
            if(debugMode==true){
                //show the threshold image after it's been "blurred"
                imshow("Final Threshold Image",thresholdImage);
            }
            else {
                //if not in debug mode, destroy the windows so we don't see them anymore
                cv::destroyWindow("Final Threshold Image");
            }
            
            //if tracking enabled, search for contours in our thresholded image
            if(trackingEnabled){
                
                searchForMovement(thresholdImage, cropFrame1);
            }
            //show our captured frame
            imshow("Frame1",cropFrame1);

            switch(waitKey(10)){
                    
                case 27: //'esc' key has been pressed, exit program.
                    return 0;
                case 116: //'t' has been pressed. this will toggle tracking
                    trackingEnabled = !trackingEnabled;
                    if(trackingEnabled == false) cout<<"Tracking disabled."<<endl;
                    else cout<<"Tracking enabled."<<endl;
                    break;
                case 100: //'d' has been pressed. this will debug mode
                    debugMode = !debugMode;
                    if(debugMode == false) cout<<"Debug mode disabled."<<endl;
                    else cout<<"Debug mode enabled."<<endl;
                    break;
                case 112: //'p' has been  pressed. this will pause/resume the code.
                    pause = !pause;
                    if(pause == true){ cout<<"Code paused, press 'p' again to resume"<<endl;
                        while (pause == true){
                            //stay in this loop until 
                            switch (waitKey()){
                                    //a switch statement inside a switch statement? Mind blown.
                                case 112: 
                                    //change pause back to false
                                    pause = false;
                                    cout<<"Code resumed."<<endl;
                                    break;
                            }
                            
                        }
                    }
                    
                    
            }
            
            
        }
        //release the capture before re-opening and looping again.
        capture.release();
    }
    
    return 0;
    
}