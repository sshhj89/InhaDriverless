#include <opencv2/highgui.hpp>
#include <opencv2/tracking.hpp>
#include "MetaHeader.h"

#define drawCross( center, color, d )                                 \
line( img, Point( center.x - d, center.y - d ), Point( center.x + d, center.y + d ), color, 2, CV_AA, 0); \
line( img, Point( center.x + d, center.y - d ), Point( center.x - d, center.y + d ), color, 2, CV_AA, 0 )

using namespace cv;
using namespace std;


static Point mousePos;

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

int main( )
{
    
    KalmanFilter KF(4, 2, 0);
   
    namedWindow("My Window", 1);
    setMouseCallback("My Window", CallBackFunc, NULL);

    KF.transitionMatrix = (Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1); //A
    Mat_<float> measurement(2,1); //B
    measurement.setTo(Scalar(0));
    
    KF.statePre.at<float>(0) = mousePos.x;
    KF.statePre.at<float>(1) = mousePos.y;
    KF.statePre.at<float>(2) = 0;
    KF.statePre.at<float>(3) = 0;
    setIdentity(KF.measurementMatrix);  //set each noise
    setIdentity(KF.processNoiseCov, Scalar::all(1e-4));
    setIdentity(KF.measurementNoiseCov, Scalar::all(10));
    setIdentity(KF.errorCovPost, Scalar::all(.1));
    // Image to show mouse tracking
    Mat img(600, 800, CV_8UC3);
    vector<Point> mousev,kalmanv;
    mousev.clear();
    kalmanv.clear();
    
    while(1)
    {
        // First predict, to update the internal statePre variable
        Mat prediction = KF.predict();
        Point predictPt(prediction.at<float>(0),prediction.at<float>(1));
        
        // Get mouse point

        measurement(0) = mousePos.x;
        measurement(1) = mousePos.y;
//
        // The update phase
        Mat estimated = KF.correct(measurement);
        
        Point statePt(estimated.at<float>(0),estimated.at<float>(1));
        Point measPt(measurement(0),measurement(1));
        // plot points
        //imshow("mouse kalman", img);
        img = Scalar::all(0);
        
        mousev.push_back(measPt);
        kalmanv.push_back(statePt);
        drawCross( statePt, Scalar(255,255,255), 5 );
        drawCross( measPt, Scalar(0,0,255), 5 );
        
        for (int i = 0; i < mousev.size()-1; i++)
            line(img, mousev[i], mousev[i+1], Scalar(255,255,0), 1);
        
        for (int i = 0; i < kalmanv.size()-1; i++) 
            line(img, kalmanv[i], kalmanv[i+1], Scalar(0,155,255), 1);
        
        imshow("My Window",img);
        
        waitKey(10);  
    }
    
    return 0;
}