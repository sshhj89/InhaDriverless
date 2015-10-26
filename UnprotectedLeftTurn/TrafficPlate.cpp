//
//  TrafficPlate.cpp
//  UnprotectedLeftTurn
//
//  Created by SonHojun on 10/18/15.
//  Copyright (c) 2015 SonHojun. All rights reserved.
//

#include "TrafficPlate.h"


void TrafficPlate::findCircles(const Mat& image)
{
    this->circles.clear();
    
    if(image.channels() == 3)
        cvtColor( image, image, CV_BGR2GRAY );
    
    GaussianBlur( image, image, Size(5, 5), 1.5 );
    HoughCircles( image, circles, CV_HOUGH_GRADIENT, 2, 20, 200, 10, 2, 7);

}
void TrafficPlate::drawCircles(const Mat& image)
{
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( image, center, 3, Scalar(0,255,0), -1, 8, 0 );
        circle( image, center, radius, Scalar(255,0,0), 3, 8, 0 );
    }
    
    //imshow("circle",image);
}

void TrafficPlate::findSquares( const Mat& image)
{
    this->squares.clear();
    int  N = 3;
    Mat pyr, timg, gray0(image.size(), CV_8U), gray;
    image.copyTo(timg);
    // down-scale and upscale the image to filter out the noise
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());

    //   imshow("pty",timg);
    
    vector<vector<Point> > contours;
    for( int c = 1; c < 3; c++ ) //
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);
        
        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            if( l == 0 )
            {
                Canny(gray0, gray, 100,250, 3);
                dilate(gray, gray, Mat(), Point(-1,-1)); // remove hole
//                if(l == 0)
//                    imshow("l0",gray);
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
                
//                if(l ==1)
//                    imshow("l1",gray);
//                else if(l ==2)
//                    imshow("l2",gray);
//                else if(l == 3)
//                    imshow("l2",gray);
//                else if(l == 4)
//                    imshow("l3",gray);
//                else if(l == 5)
//                    imshow("l4",gray);
//                else if(l == 6)
//                    imshow("l5",gray);
//                else if(l == 7)
//                    imshow("l6",gray);
//                else if(l == 8)
//                    imshow("l7",gray);
//                else if(l == 9)
//                    imshow("l8",gray);
//                else if(l == 10)
//                    imshow("l9",gray);
            }
            
            // find contours and store them all as a listy
            findContours(gray, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//            Mat drawing = Mat::zeros( gray.size(), CV_8UC3 );
//            for( int i = 0; i< contours.size(); i++ )
//            {
//                Scalar color = Scalar( i+150, i+150, i+150);
//                drawContours( drawing, contours, i, color);
//            }
//            imshow("contour",drawing);
            
            vector<Point> approx;
            
            for( size_t i = 0; i < contours.size(); i++ )
            {
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.1, true);
                
                if( approx.size() == 4 &&
                   fabs(contourArea(Mat(approx))) > 100 && fabs(contourArea(Mat(approx))) < 600 &&
                   isContourConvex(Mat(approx)) )
                {
                    double maxCosine = 0;
                    
                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }
                    
                    if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }
            }
            
            cout<<l << " size squares: " << squares.size()<<endl;
            
            
        }
        
        cout<<"==============="<<endl;
        
        
    }
    
}
void TrafficPlate::drawSquares( Mat& image)
{
   
    for( size_t i = 0; i < this->squares.size(); i++ )
    {
        const Point* p = &this->squares[i][0];
        int n = (int)this->squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(255,255,255),3,LINE_AA);
    }
    //imshow("sq", image);

}

void TrafficPlate::drawTraffic(Mat& image)
{
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( image, center, 3, Scalar(0,255,0), -1, 8, 0 );
        circle( image, center, radius, Scalar(255,0,0), 3, 8, 0 );
    }
    
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,255,0),3,LINE_AA);
    }
    
    //imshow("traffic",image);
}

void TrafficPlate::debugSquares( vector<vector<Point> >squares, Mat& image )
{
    for ( int i = 0; i< squares.size(); i++ ) {
        // draw contour
        //cv::drawContours(image, squares, i, cv::Scalar(255,0,0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        
        // draw bounding rect
        cv::Rect rect = boundingRect(cv::Mat(squares[i]));
        cv::rectangle(image, rect.tl(), rect.br(), cv::Scalar(255,255,255), 2, 8, 0);
        
//        // draw rotated rect
//        cv::RotatedRect minRect = minAreaRect(cv::Mat(squares[i]));
//        cv::Point2f rect_points[4];
//        minRect.points( rect_points );
//        for ( int j = 0; j < 4; j++ ) {
//            //cv::line( image, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255), 1, 8 ); // blue
//        }
    }
    
    
}



