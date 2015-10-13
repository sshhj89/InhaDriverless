#include "opencv2/opencv.hpp"
#include "opencv/cv.h"
#include <opencv2/highgui/highgui.hpp>
#include "CTracker.h"
#include <iostream>
#include <vector>

using namespace cv;
using namespace std;
Mat fgMaskMOG2;
Ptr<BackgroundSubtractor> pMOG2; 
Scalar Colors[]={Scalar(255,0,0),Scalar(0,255,0),Scalar(0,0,255),Scalar(255,255,0),Scalar(0,255,255),Scalar(255,0,255),Scalar(255,127,255),Scalar(127,0,255),Scalar(127,0,127)};


int main(int ac, char** av)
{

   Mat frame,thresh_frame;
  vector<Mat> channels;

 
  vector<Vec4i> hierarchy;
  vector<vector<Point> > contours;
  

  vector<Point2f> centers;
  



    cv::Mat fore;

		

    cv::BackgroundSubtractorMOG2 bg;

    //bg.nmixtures = 3;
    //bg.bShadowDetection = false;
    int incr=0;

    int track=0;
	VideoCapture cap;

    cap.open("C:\\Users\\경철\\Desktop\\전방.mp4");

  if(!cap.isOpened())
  {
    cerr << "Problem opening video source" << endl;
  }
  


  CTracker tracker(0.2,0.5,60.0,10,20);
   

  while((char)waitKey(60) != 'q' && cap.grab())
    {

   Point s, p;
  bool bSuccess =cap.retrieve(frame);


  stringstream ss;
        rectangle(frame, cv::Point(10, 2), cv::Point(100,20),
            cv::Scalar(255,255,255), -1);
        ss << cap.get(CV_CAP_PROP_POS_FRAMES);
		
        string frameNumberString = ss.str();
		
        putText(frame, frameNumberString.c_str(), cv::Point(15, 15),
            FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,0,0));
		centers.clear();
  // read a new frame from video

			
        if (!bSuccess) //if not success, break loop
       {
             cout << "ERROR: Cannot read a frame from video file" << endl;
             break;
        }
        bg.operator ()(frame,fore);
		
		threshold(fore,fore,127,255,CV_THRESH_BINARY);
		medianBlur(fore,fore,3);
		
        erode(fore,fore,Mat());
        erode(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
		 dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());
        dilate(fore,fore,Mat());

		
		//Canny(fore, edges, 10, 100, 3);
	  // imshow("contours",edges);


        cv::normalize(fore, fore, 0, 1., cv::NORM_MINMAX);
        cv::threshold(fore, fore, .5, 1., CV_THRESH_BINARY);


      split(frame, channels);
      add(channels[0], channels[1], channels[1]);
      subtract(channels[2], channels[1], channels[2]);
      threshold(channels[2], thresh_frame, 50, 255, CV_THRESH_BINARY);
      medianBlur(thresh_frame, thresh_frame, 5);

	  
	  
      findContours(fore, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	 
      vector<vector<Point> > contours_poly( contours.size() );
      vector<Rect> boundRect( contours.size() );

      Mat drawing = Mat::zeros(thresh_frame.size(), CV_8UC1);
      for(size_t i = 0; i < contours.size(); i++)
        {
//          cout << contourArea(contours[i]) << endl;
          if(contourArea(contours[i]) > 500)
            drawContours(drawing, contours, i, Scalar::all(255), CV_FILLED, 8, vector<Vec4i>(), 0, Point());
        }
	
      thresh_frame = drawing;

        for( size_t i = 0; i < contours.size(); i++ )
       {
		   approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
         boundRect[i] = boundingRect( Mat(contours_poly[i]) );

     }

      for( size_t i = 0; i < contours.size(); i++ )
       {
           if(contourArea(contours[i]) > 1000){
         rectangle( frame, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 2, 8, 0 );
		 // Rect r = boundRect[i];
        //  frame(r).copyTo(images_array);  
		// img_array.push_back(images_array);
        Point center = Point(boundRect[i].x + (boundRect[i].width /2), boundRect[i].y + (boundRect[i].height/2));
        cv::circle(frame,center, 8, Scalar(0, 0, 255), -1, 1,0);
		centers.push_back(center);
		   }
	  }


	if(centers.size()>0)
	{
		tracker.Update(centers);
		
		cout << tracker.tracks.size()  << endl;

		for(int i=0;i<tracker.tracks.size();i++)
		{
			if(tracker.tracks[i]->trace.size()>1)
			{
				for(int j=0;j<tracker.tracks[i]->trace.size()-1;j++)
				{
					line(frame,tracker.tracks[i]->trace[j],tracker.tracks[i]->trace[j+1],Colors[tracker.tracks[i]->track_id%9],2,CV_AA);
					//drawCross(frame, tracker.tracks[i]->trace[j], Scalar(255, 255, 255), 5);
				}
			}
		}
	}

	
	imshow("OUTPUT Fist Camera",frame);
	

	waitKey(30);
	}
	//delete detector;
	//destroyAllWindows();
	return 0;

}
