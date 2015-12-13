#include "CTracker.h"
#include "kalman.h"
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;
Mat fgMaskMOG2;
Ptr<BackgroundSubtractor> pMOG2;
//Scalar Colors[]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,255,0),Scalar(0,255,255),Scalar(255,0,255),Scalar(255,127,255),Scalar(127,0,255),Scalar(127,0,127)};

TKalmanFilter::TKalmanFilter(Point2f pt,float dt,float Accel_noise_mag)
{
    //time increment (lower values makes target more "massive")
    deltatime = dt; //0.2
    
    // We don't know acceleration, so, assume it to process noise.
    // But we can guess, the range of acceleration values thich can be achieved by tracked object.
    // Process noise. (standard deviation of acceleration: ??2)
    // shows, woh much target can accelerate.
    //float Accel_noise_mag = 0.5;
    
    //4 state variables, 2 measurements
    kalman = new KalmanFilter( 4, 2, 0 );
    // Transition matrix
    kalman->transitionMatrix = (Mat_<float>(4, 4) << 1,0,deltatime,0,   0,1,0,deltatime,  0,0,1,0,  0,0,0,1);
    
    // init...
    LastResult = pt;
    kalman->statePre.at<float>(0) = pt.x; // x
    kalman->statePre.at<float>(1) = pt.y; // y
    
    kalman->statePre.at<float>(2) = 0;
    kalman->statePre.at<float>(3) = 0;
    
    kalman->statePost.at<float>(0)=pt.x;
    kalman->statePost.at<float>(1)=pt.y;
    
    setIdentity(kalman->measurementMatrix);
    
    kalman->processNoiseCov=(Mat_<float>(4, 4) <<
                             pow(deltatime,4.0)/4.0	,0						,pow(deltatime,3.0)/2.0		,0,
                             0						,pow(deltatime,4.0)/4.0	,0							,pow(deltatime,3.0)/2.0,
                             pow(deltatime,3.0)/2.0	,0						,pow(deltatime,2.0)			,0,
                             0						,pow(deltatime,3.0)/2.0	,0							,pow(deltatime,2.0));
    
    
    kalman->processNoiseCov*=Accel_noise_mag;
    
    setIdentity(kalman->measurementNoiseCov, Scalar::all(0.1));
    
    setIdentity(kalman->errorCovPost, Scalar::all(.1));
    
}
//---------------------------------------------------------------------------
TKalmanFilter::~TKalmanFilter()
{
    delete kalman;
}

//---------------------------------------------------------------------------
Point2f TKalmanFilter::GetPrediction()
{
    Mat prediction = kalman->predict();
    LastResult=Point2f(prediction.at<float>(0),prediction.at<float>(1));
    return LastResult;
}
//---------------------------------------------------------------------------
Point2f TKalmanFilter::Update(Point2f p, bool DataCorrect)
{
    Mat measurement(2,1,CV_32FC1);
    if(!DataCorrect)
    {
        measurement.at<float>(0) = LastResult.x;  //update using prediction
        measurement.at<float>(1) = LastResult.y;
    }
    else
    {
        measurement.at<float>(0) = p.x;  //update using measurements
        measurement.at<float>(1) = p.y;
    }
    // Correction
    Mat estimated = kalman->correct(measurement);
    LastResult.x=estimated.at<float>(0);   //update using measurements
    LastResult.y=estimated.at<float>(1);
    return LastResult;
}


//int main(int ac, char** av)
//{
//    
//    Mat frame,thresh_frame;
//    vector<Mat> channels;
//    
//    
//    vector<Vec4i> hierarchy;
//    vector<vector<Point> > contours;
//    
//    
//    vector<Point2f> centers;
//    Mat fore;
//    
//    Ptr<BackgroundSubtractorMOG2> bg =   createBackgroundSubtractorMOG2();
//    
//    //bg.nmixtures = 3;
//    //bg.bShadowDetection = false;
//    int incr=0;
//    
//    int track=0;
//    VideoCapture cap;
//    
//    cap.open("/Users/sonhojun/Downloads/data/plate2.mp4");
//    
//    if(!cap.isOpened())
//    {
//        cerr << "Problem opening video source" << endl;
//    }
//    
//    
//    
//    CTracker tracker(0.2,0.5,60.0,10,20);
//    
//    
//    while((char)waitKey(60) != 'q' && cap.grab())
//    {
//        
//        Point s, p;
//        bool bSuccess =cap.retrieve(frame);
//        
////        
////        stringstream ss;
////        rectangle(frame, cv::Point(10, 2), cv::Point(100,20),
////                  cv::Scalar(255,255,255), -1);
////        ss << cap.get(CV_CAP_PROP_POS_FRAMES);
////        
////        string frameNumberString = ss.str();
////        
////        putText(frame, frameNumberString.c_str(), cv::Point(15, 15),
////                FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,0,0));
//        centers.clear();
//        // read a new frame from video
//        
//        
//        if (!bSuccess) //if not success, break loop
//        {
//            cout << "ERROR: Cannot read a frame from video file" << endl;
//            break;
//        }
//        bg->apply(frame,fore);
//        
//        threshold(fore,fore,127,255,CV_THRESH_BINARY);
//        medianBlur(fore,fore,3);
//        
//        erode(fore,fore,Mat());
//        erode(fore,fore,Mat());
//        dilate(fore,fore,Mat());
//        dilate(fore,fore,Mat());
//        dilate(fore,fore,Mat());
//        dilate(fore,fore,Mat());
//        dilate(fore,fore,Mat());
//        dilate(fore,fore,Mat());
//        dilate(fore,fore,Mat());
//        dilate(fore,fore,Mat());
//        dilate(fore,fore,Mat());
//        dilate(fore,fore,Mat());
//        dilate(fore,fore,Mat());
//        
//        Mat edges;
//        Canny(fore, edges, 10, 100, 3);
//        imshow("contours",edges);
//        
//        
//        normalize(fore, fore, 0, 1., NORM_MINMAX);
//        threshold(fore, fore, .5, 1., CV_THRESH_BINARY);
//        imshow("trhes",fore);
//        
//        split(frame, channels);
//        add(channels[0], channels[1], channels[1]);
//        subtract(channels[2], channels[1], channels[2]);
//        threshold(channels[2], thresh_frame, 50, 255, CV_THRESH_BINARY);
//        medianBlur(thresh_frame, thresh_frame, 5);
//        
//        imshow("trhesdd",thresh_frame);
//        
//        findContours(fore, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
//        
//        vector<vector<Point> > contours_poly( contours.size() );
//        vector<Rect> boundRect( contours.size() );
//        
//        Mat drawing = Mat::zeros(thresh_frame.size(), CV_8UC1);
//        for(size_t i = 0; i < contours.size(); i++)
//        {
//            //          cout << contourArea(contours[i]) << endl;
//            if(contourArea(contours[i]) > 500)
//                drawContours(drawing, contours, i, Scalar::all(255), CV_FILLED, 8, vector<Vec4i>(), 0, Point());
//        }
//        
//        thresh_frame = drawing;
//        imshow("contour",thresh_frame);
//        for( size_t i = 0; i < contours.size(); i++ )
//        {
//            approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
//            boundRect[i] = boundingRect( Mat(contours_poly[i]) );
//        }
//        
//        for( size_t i = 0; i < contours.size(); i++ )
//        {
//            if(contourArea(contours[i]) > 1000){
//                rectangle( frame, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 2, 8, 0 );
//                // Rect r = boundRect[i];
//                //  frame(r).copyTo(images_array);
//                // img_array.push_back(images_array);
//                Point center = Point(boundRect[i].x + (boundRect[i].width /2), boundRect[i].y + (boundRect[i].height/2));
//                circle(frame,center, 8, Scalar(0, 0, 255), -1, 1,0);
//                centers.push_back(center);
//            }
//        }
//        
//        
//        if(centers.size()>0)
//        {
//            tracker.Update(centers);
//            
//            cout << tracker.tracks.size()  << endl;
//            
//            for(int i=0;i<tracker.tracks.size();i++)
//            {
//                if(tracker.tracks[i]->trace.size()>1)
//                {
//                    for(int j=0;j<tracker.tracks[i]->trace.size()-1;j++)
//                    {
//                        line(frame,tracker.tracks[i]->trace[j],tracker.tracks[i]->trace[j+1],Colors[tracker.tracks[i]->track_id%9],2,CV_AA);
//                        //drawCross(frame, tracker.tracks[i]->trace[j], Scalar(255, 255, 255), 5);
//                    }
//                }
//            }
//        }
//        
//        
//        imshow("OUTPUT Fist Camera",frame);
//        
//        
//        waitKey(30);
//    }
//    //delete detector;
//    //destroyAllWindows();
//    return 0;
//    
//}