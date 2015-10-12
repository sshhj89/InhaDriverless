//
//  DriverlessCar.cpp
//  UnprotectedLeftTurn
//
//  Created by SonHojun on 8/29/15.
//  Copyright (c) 2015 SonHojun. All rights reserved.
//

#include "DriverlessCar.h"
#define THRESHOLD_LANE 25.0

void DriverlessCar::drawSquares( UMat& image, const vector<vector<Point> >& squares )
{
    Mat temp;
    image.copyTo(temp);
    
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(temp, &p, &n, 1, true, Scalar(0,255,0), 3, LINE_AA);
    }
    imshow("sq", temp);
}

void DriverlessCar::findSquares( const UMat& image, vector<vector<Point> >& squares )
{
    squares.clear();
    int thresh = 50, N = 11;
    
    Mat pyr, timg, gray0(image.size(), CV_8U), gray;
    image.copyTo(timg);
    // down-scale and upscale the image to filter out the noise
    //pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    //pyrUp(pyr, timg, image.size());
    //imshow("pty",timg);
    vector<vector<Point> > contours;
    
    // find squares in every color plane of the image
    for( int c = 0; c < 3; c++ )
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);
        
        
        // try several threshold levels
        for( int l = 0; l < N; l++ )
        {
            // hack: use Canny instead of zero threshold level.
            // Canny helps to catch squares with gradient shading
            if( l == 0 )
            {
                // apply Canny. Take the upper threshold from slider
                // and set the lower to 0 (which forces edges merging)
                Canny(gray0, gray, 20,250, 3);
                // dilate canny output to remove potential
                // holes between edge segments
                //dilate(gray, gray, Mat(), Point(-1,-1));
                //imshow("edge",gray);
            }
            else
            {
                // apply threshold if l!=0:
                //     tgray(x,y) = gray(x,y) < (l+1)*255/N ? 255 : 0
                gray = gray0 >= (l+1)*255/N;
            }
            
            // find contours and store them all as a list
            findContours(gray, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
            vector<Point> approx;
            
            // test each contour
            for( size_t i = 0; i < contours.size(); i++ )
            {

                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
                
                if( approx.size() == 4 &&
                   fabs(contourArea(Mat(approx))) > 100 && fabs(contourArea(Mat(approx))) < 300 &&
                   isContourConvex(Mat(approx)) )
                {
                    double maxCosine = 0;
                    
                    for( int j = 2; j < 5; j++ )
                    {
                        // find the maximum cosine of the angle between joint edges
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }
                    
                    // if cosines of all angles are small
                    // (all angles are ~90 degree) then write quandrange
                    // vertices to resultant sequence
                    if( maxCosine < 0.3 )
                        squares.push_back(approx);
                }
            }
        }
    }
}

void DriverlessCar::findCircles(const UMat& image, vector<Vec3f>& circles)
{
    Mat temp;
    image.copyTo(temp);
    circles.clear();
    
    if(image.channels() == 3)
        cvtColor( image, temp, CV_BGR2GRAY );
    
    GaussianBlur( temp, temp, Size(5, 5), 1.5 );
    HoughCircles( temp, circles, CV_HOUGH_GRADIENT, 2, 20, 200, 10, 1, 5);
}

void DriverlessCar::drawCircles(const cv::UMat &image, const vector<Vec3f> &circles)
{
    Mat temp;
    image.copyTo(temp);
    
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( temp, center, 3, Scalar(0,255,0), -1, 8, 0 );
        circle( temp, center, radius, Scalar(255,0,0), 3, 8, 0 );
    }
    
    imshow("circle",temp);
    
}

void DriverlessCar::drawTraffic(const cv::UMat &image, const vector<vector<Point> > &squares,
                                const vector<Vec3f> &circles)
{
    Mat temp;
    image.copyTo(temp);
    
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( temp, center, 3, Scalar(0,255,0), -1, 8, 0 );
        circle( temp, center, radius, Scalar(255,0,0), 3, 8, 0 );
    }
    
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(temp, &p, &n, 1, true, Scalar(0,255,0), 3, LINE_AA);
    }

    
    imshow("traffic",temp);
    
}

int DriverlessCar::laneChecker(bool rfPosi, bool rfNega)
{
    
    if(this->finder.getPositiveLinesInfo().size() != 2) // need for optimize .
        return 0;
    
    double minBandRight = 1000.;
    double minBandLeft = 1000.;
    vector<LineInfo>::const_iterator it_posi = this->finder.getPositiveLinesInfo().begin();
    vector<LineInfo>::const_iterator it_nega = this->finder.getNagativeLinesInfo().begin();
    while(it_posi != this->finder.getPositiveLinesInfo().end())
    {
//        if(rfPosi == false) need fix
//            return 0;
        
        if((*it_posi).getRho() < minBandLeft)
        {
            this->setBkBandLeft(this->getBandLeft());
            minBandLeft = (*it_posi).getRho();
            this->setBandLeft(minBandLeft);
        }
        
        it_posi++;
    }
    
    while(it_nega != this->finder.getNagativeLinesInfo().end())
    {
//        if(rfNega == false)  need fix
//            return 0;
        
        if((*it_nega).getRho() < minBandRight)
        {
            this->setBkBandRight(this->getBandRight());
            minBandRight = (*it_nega).getRho();
            this->setBandRight(minBandRight);
        }
        
        it_nega++;
    }

    //cout<<"bandLEft : " << this->getBandLeft() << " bkBandLEft: " <<this->getBkBandLeft()<<endl;
   // cout<<"bandRight : " << this->getBandRight() << " bkBandRight: " <<this->getBkBandRight()<<endl;

    
    if(this->getBandLeft() - this->getBkBandLeft() > THRESHOLD_LANE )
    {
        if(this->getBkBandLeft() == -1.)
            return 0;

        cout<<"move left posi" <<endl;

        return -1;
    }else if(this->getBkBandRight() - this->getBandRight() > THRESHOLD_LANE )
    {
        if(this->getBkBandRight() == -1.)
            return 0;
        
        cout<<"move left nega" <<endl;

        return -1;
    }
    else if(this->getBandRight() - this->getBkBandRight() > THRESHOLD_LANE )
    {
        if(this->getBkBandRight() == -1.)
            return 0;
        
        cout<<"move Right posi " <<endl;

        return 1;
    }else if(this->getBkBandLeft() - this->getBandLeft() > THRESHOLD_LANE )
    {
        if(this->getBkBandLeft() == -1.)
            return 0;
        
        cout<<"move right posi" <<endl;

        return 1;
    }

    
    
 
    return 0;
}
