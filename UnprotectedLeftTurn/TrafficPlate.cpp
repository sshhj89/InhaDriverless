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
    if(image.channels() == 3)
        cvtColor( image, image, CV_BGR2GRAY );
    
    GaussianBlur( image, image, Size(5, 5), 1.5 );
    
    if(seqImage == false)
    {
       // this->prevCircles.clear();
        HoughCircles( image, prevCircles, CV_HOUGH_GRADIENT, 2, 20, 200, 10, 2, 7);
        cout<<"call1"<<endl;
    }
    else if (seqImage == true)
    {
       // this->curCircles.clear();
        HoughCircles( image, curCircles, CV_HOUGH_GRADIENT, 2, 20, 200, 10, 2, 7);
        cout<<"call2"<<endl;
        filterLights();
    }
    
}

void TrafficPlate::drawCircles(const Mat& image)
{
    list<Point>::const_iterator tpl_it = tpl.begin();
    list<int>::const_iterator tplr_it = tplR.begin();
    
    while(tpl_it != tpl.end())
    {
        Point center(cvRound((*tpl_it).x), cvRound((*tpl_it).y));
        int radius = cvRound((*tplr_it));
        circle( image, center, 3, Scalar(0,255,0), -1, 8, 0 );
        circle( image, center, radius, Scalar(255,0,0), 3, 8, 0 );
        tpl_it++;
        tplr_it++;
        
    }
}

void TrafficPlate::filterLights()
{
    
    tpl.clear();
    tplR.clear();
 
    for( size_t i = 0; i < prevCircles.size(); i++ )
    {
        Point prevCenter(cvRound(prevCircles[i][0]), cvRound(prevCircles[i][1]));
        cout<<"prev center: "<< prevCenter.x <<", "<<prevCenter.y<<endl;

        for( size_t j = 0; j < curCircles.size(); j++ )
        {
            Point curCenter(cvRound(curCircles[j][0]), cvRound(curCircles[j][1]));
            double ud = sqrt(pow((curCenter.x - prevCenter.x),2) + pow((curCenter.y - prevCenter.y),2));
            
            if(0 < ud && ud <= 5)
            {
                cout<<"ud : " << ud<<endl;
                cout<<"curCenter: "<< curCenter.x <<", "<<curCenter.y<<endl;
                tpl.push_back(curCenter);
                tplR.push_back(cvRound(curCircles[i][2]));
                //cvWaitKey(1000);
            }
            else if(ud == 0 && totalDiff[0] < 1.3)
            {
                cout<<"car stop??" <<endl;
                cout<<"curCenter: "<< curCenter.x <<", "<<curCenter.y<<endl;
                tpl.push_back(curCenter);
                tplR.push_back(cvRound(curCircles[i][2]));
            }
        }
       // cvWaitKey(1000);
    }
    
}


void TrafficPlate::findSquares( Mat& image)
{
    this->squares.clear();
    int  N = 3;
    Mat pyr, timg, gray0(image.size(), CV_8U), gray;
    image.copyTo(timg);

       
    pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
    pyrUp(pyr, timg, image.size());
    vector<vector<Point> > contours;
    for( int c = 1; c < 3; c++ ) //
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);
        Mat dividedCh;
        gray0.copyTo(dividedCh);
       
//        if( c== 1)
//            imshow("ch1",dividedCh);
//        else
//            imshow("ch2", dividedCh);

        for( int l = 0; l < N; l++ )
        {
            
            if( l == 0 )
            {
               
                Canny(gray0, gray, 100,350, 3);
                
                dilate(gray, gray, Mat(), Point(-1,-1)); // remove hole
               // imshow("gray0",gray);
              
            }
            else
            {
                
                gray = gray0 >= (l+1)*255/N;
             //   bitwise_not(gray, lights, gray);
//                if(l == 1)
//                    imshow("l1",gray);
//                else if (l ==2)
//                    imshow("l2",gray);
//                else if(l==3)
//                    imshow("l3",gray);
//                else if(l==4)
//                    imshow("l4",gray);
//                else
//                    imshow("l5",gray);
                    
            }
            
            // find contours and store them all as a listy
            findContours(gray, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
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
        }
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
    
    list<Point>::const_iterator tpl_it = tpl.begin();
    list<int>::const_iterator tplr_it = tplR.begin();
    vector<vector<Point>> newsquares;
    
    while(tpl_it != tpl.end())
    {
        Point center(cvRound((*tpl_it).x), cvRound((*tpl_it).y));
        int radius = cvRound((*tplr_it));
        
        for( size_t i = 0; i < squares.size(); i++ )
        {
            Rect rect = boundingRect(Mat(squares[i]));
            
            cout<<"Rect x,y : " <<rect.tl().x <<" , " <<rect.tl().y<<endl;
            cout<<"Rect2 x2,y2: " <<rect.br().x <<" , " <<rect.br().y<<endl;
            cout<<"curCenter: "<< center.x <<", "<<center.y<<endl;


            if( center.y > rect.tl().y && center.y < rect.br().y)
            {
                rectangle(image, rect.tl(), rect.br(), Scalar(255,255,255), 2, 8, 0);
                circle( image, center, 3, Scalar(0,255,0), -1, 8, 0 );
                circle( image, center, radius, Scalar(255,0,0), 3, 8, 0 );
                //cvWaitKey(1000);
                /// traffic find also find plate or traffic
            }
        }
        
        tpl_it++;
        tplr_it++;
        
    }
    
    
    //imshow("traffic",image);
}

void TrafficPlate::debugSquares( vector<vector<Point> >squares, Mat& image )
{
    for ( int i = 0; i< squares.size(); i++ )
    {
        Rect rect = boundingRect(Mat(squares[i]));
        rectangle(image, rect.tl(), rect.br(), Scalar(255,255,255), 2, 8, 0);
    }
}

void TrafficPlate::featureDetect(Mat& image, vector<vector<Point>> squares)
{
    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 400;
    vector<KeyPoint> keypointsO; //keypoints for object
    vector<KeyPoint> keypointsS; //keypoints for scene
    Mat db_original = imread("/Users/sonhojun/Downloads/signals2.png");
  //  equalizeHist(db_original, db_original);
    Mat tmp;
    
    GaussianBlur(db_original, tmp, cv::Size(5,5), 5);
    addWeighted(db_original, 1.5, tmp, -0.5, 0, db_original);
    
    
    Mat descriptors_object, descriptors_scene;
    
   Ptr<xfeatures2d::SURF> surf = xfeatures2d::SURF::create(2500);
    
    surf->detect(db_original,keypointsO);
    surf->compute( db_original, keypointsO, descriptors_object );
    
    FlannBasedMatcher matcher = FlannBasedMatcher();
    //BFMatcher matcher = BFMatcher();
    vector< DMatch > matches;
    std::vector< std::vector<cv::DMatch> > os_matches, so_matches;
    
    for ( int i = 0; i< squares.size(); i++ )
    {
        Rect rect = boundingRect(Mat(squares[i]));

        if((abs(rect.tl().x - rect.br().x)/abs(rect.tl().y - rect.br().y)) > 1.1)
            continue;
        cout<<"Rect x,y : " <<rect.tl().x <<" , " <<rect.tl().y<<endl;
        Mat data = image(Rect(rect.tl(),rect.br()));
        
        
        cvtColor(data, data, CV_BGR2GRAY);
        Mat exp1;
        resize(data, exp1, Size(db_original.cols,db_original.rows));
       // pyrUp(data, exp1, Size(data.cols*2,data.rows*2));
        
        equalizeHist(exp1, exp1);
        Mat tmp;
        cv::GaussianBlur(exp1, tmp, cv::Size(5,5), 5);
        cv::addWeighted(exp1, 1.5, tmp, -0.5, 0, exp1);
        
        threshold(exp1, exp1, 50, 255, THRESH_BINARY_INV);
        imshow("cropsqu",exp1);
        
        surf->detect(exp1,keypointsS);
        surf->compute(exp1, keypointsS, descriptors_scene );
        
//        cout<<exp1.cols<<" , "<<exp1.rows<<endl;
//        cout<<descriptors_object.size << " , " << descriptors_scene.size << endl;
        if(descriptors_scene.rows == 0 || descriptors_scene.rows == 1)
            continue;
        
        
        matcher.match(descriptors_object, descriptors_scene, matches);
//       matcher.knnMatch(descriptors_object, descriptors_scene, os_matches, 2);
        
        double max_dist = 0; double min_dist = 100;
        
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < descriptors_object.rows; i++ )
        { double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }
//        
        printf("-- Max dist : %f \n", max_dist );
        printf("-- Min dist : %f \n", min_dist );
//        
        //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
        //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
        //-- small)
        //-- PS.- radiusMatch can also be used here.
        std::vector< DMatch > good_matches;
        
        for( int i = 0; i < descriptors_object.rows; i++ )
        { if( matches[i].distance <= max(2*min_dist, 0.02) )
        { good_matches.push_back( matches[i]); }
        }
        
        //-- Draw only "good" matches
        Mat img_matches;
        if(keypointsS.size() == 0 )
            continue;
        drawMatches( db_original, keypointsO, exp1, keypointsS,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        
        //-- Show detected matches
        imshow( "Good Matches", img_matches );
//        
//        for( int i = 0; i < (int)good_matches.size(); i++ )
//        { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }
//
       
//        Mat img_matches;
//        drawMatches(db_original, keypointsO, exp1, keypointsS, os_matches, img_matches);
//        imshow("matches", img_matches);
//        
     
        
        
        //waitKey(0);
    }
}





