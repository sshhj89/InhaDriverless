//
//  TrafficPlate.cpp
//  UnprotectedLeftTurn
//
//  Created by SonHojun on 10/18/15.
//  Copyright (c) 2015 SonHojun. All rights reserved.
//

#include "TrafficPlate.h"
#define frameNum 10;

void TrafficPlate::splitAsGreenBin(const Mat &image)
{
    Mat image_r( image.rows, image.cols, CV_8UC1);
    Mat image_g( image.rows, image.cols, CV_8UC1);
    Mat image_b( image.rows, image.cols, CV_8UC1);
    
    Mat out[] = { image_b, image_g, image_r };
    int from_to[] = { 0,0, 1,1, 2,2 };
    mixChannels( &image, 1, out, 3, from_to, 3 );
    
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;
    
    minMaxLoc( image_g, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    
 //   cout<<"maxVal: "<<maxVal<<"maxLoc: "<<maxLoc<<endl;
    double avgGreen = (sum(image_g) / (image_g.cols * image_g.rows))[0];
    Scalar     mean;
    Scalar     stddev;
    
    cv::meanStdDev ( image_g, mean, stddev );
    int       mean_pxl = mean.val[0];
    int     stddev_pxl = stddev.val[0];
    
    int dev = sqrt(stddev_pxl);
    
    threshold(image_g, image_g, mean_pxl+2*dev, 255, THRESH_BINARY);
    image_g.copyTo(greenBin);
}

Mat TrafficPlate::binaryOrigin()
{
    Mat temp;
    cvtColor(origin, temp, CV_RGB2GRAY);
    
    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;
    
    minMaxLoc( temp, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    
    Scalar     mean;
    Scalar     stddev;
    
    meanStdDev ( temp, mean, stddev );
    int       mean_pxl = mean.val[0];
    int     stddev_pxl = stddev.val[0];
    
    threshold(temp, temp, mean_pxl - 2*stddev_pxl, 255, THRESH_BINARY);

    return temp;
}

void TrafficPlate::setGreenHSV(const Mat& image)
{
    cvtColor(image, greenHsv, CV_BGR2HSV);
    cvtColor(greenHsv, greenHsv, CV_BGR2HSV);
    inRange(greenHsv,Scalar(LIGHTS_HUE_MIN, LIGHTS_SAT_MIN, LIGHTS_INT_MIN), Scalar(LIGHTS_HUE_MAX, LIGHTS_SAT_MAX, LIGHTS_INT_MAX),greenHsv);
    medianBlur(greenHsv, greenHsv, 5);
    dilate(greenHsv, greenHsv, Mat(), Point(-1,-1));
    dilate(greenHsv,greenHsv,Mat(),Point(1,1));
}

void TrafficPlate::findCircles(const Mat& image)
{
    HoughCircles( image, circles, CV_HOUGH_GRADIENT, 2, 10, 200, 10, 1, 7);
    filterLights();
}

void TrafficPlate::drawCircles( Mat& image)
{
    list<Point>::const_iterator tpl_it = tpl.begin();
    list<int>::const_iterator tplr_it = tplR.begin();
    
    while(tpl_it != tpl.end())
    {
        Point center(cvRound((*tpl_it).x), cvRound((*tpl_it).y));
        int radius = cvRound((*tplr_it));
        circle(image, center, 1, Scalar(255,255,255), -1, 5, 0 );
        circle(image, center, radius, Scalar(255,255,255), 3, 5, 0 );
        
        tpl_it++;
        tplr_it++;
    }
}

void TrafficPlate::filterLights()
{
    tpl.clear();
    tplR.clear();
   
    Mat temp;
    temp = binaryOrigin();
    
    vector<Vec3f>::const_iterator itc = circles.begin();
    int idx= 0;
    while(itc != circles.end())
    {
        Point center(cvRound(itc[idx][0]), cvRound(itc[idx][1]));
        int radius = cvRound(itc[idx][2]);
        
        tpl.push_back(center);
        tplR.push_back(radius);
        Mat ele(radius*2,radius,CV_8UC1,Scalar(0));
        
        Mat greenSquare=temp(Rect(center.x - radius*5,center.y-radius,radius*4,radius*2));  //bound check
        //imshow("ele", greenSquare);
        
        double avgGreen = ((sum(greenSquare))[0]) / 255;
        //cout<<avgGreen<<endl;
        if(avgGreen > 10)
        {
            //circles.erase(itc);
            itc++;
            continue;
        }
        
        tpl.push_back(center);
        tplR.push_back(radius);
        itc++;
    }
    
    /*
        Todo : consider two circle's y value
    */
    
    if(tpl.size() > 0)
        lightsColor = true; //green;
    else
        lightsColor = false;
    

    
    
    

}

bool TrafficPlate::filterPlate(Mat& possiblePlate)
{
    vector<vector<Point>> contours4Plate;
    findContours(possiblePlate, contours4Plate, CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    Mat result(possiblePlate.size(),CV_8U,Scalar(255));
    
    int cmin = 50;
    int cmax = 1000;
    
    vector<vector<Point>>::const_iterator itc = contours4Plate.begin();
    int maxIdx = 0;
    
    while(itc != contours4Plate.end())
    {
        if(itc->size() < cmin || itc->size() > cmax)
            itc = contours4Plate.erase(itc);
        else
        {
            maxIdx++;
            itc++;
        }
        
    }
    
    if(contours4Plate.size() !=4)
        return false;
    
    Point plateArrow;
    vector<Point> centers;
    
    if(contours4Plate.size() == 4)
    {
       // cout<<"ppossible plate: "<< maxIdx<<endl;
        for(int i=0; i< contours4Plate.size(); i++)
        {
            if( i == maxIdx-1)
            {
                vector<Point> poly;
                approxPolyDP(Mat(contours4Plate[maxIdx-1]), poly, 5, true);
                vector<Point>::const_iterator pit = poly.begin();
                while(pit != (poly.end()-1))
                {
                    line(result,*pit,*(pit+1),Scalar(0),2);
                    ++pit;
                }
                
                line(result,*(poly.begin()),*(poly.end()-1),Scalar(20),2);
                Moments mom = moments(Mat(contours4Plate[maxIdx-1]));
                plateArrow = Point(mom.m10/mom.m00,mom.m01/mom.m00);
            }
            else
            {
                float rad;
                Point2f center;
                minEnclosingCircle(Mat(contours4Plate[i]), center, rad);
                
                Moments mom = moments(Mat(contours4Plate[i]));
                circle(result, Point(mom.m10/mom.m00,mom.m01/mom.m00),2,Scalar(0),2);
                
                circle(result,Point(center),static_cast<int>(rad),Scalar(0),2);
                centers.push_back(Point(mom.m10/mom.m00,mom.m01/mom.m00));
            }
        }
        
        if(plateArrow.y > result.rows* 0.3)
            return false;
        
        bool isPlate = true;
        for(int i=0;i<centers.size();i++)
        {
            if(plateArrow.y > centers[i].y)
            {
                isPlate = false;
                break;
            }
            
            if(centers[i].y < result.rows* 0.7)
                isPlate = false;
            
            if(abs(centers[i].y - centers[((i+1) == 3)? 0:(i+1)].y) > 5)
                isPlate = false;
            
        }
        
        if(isPlate == true)
        {
            drawContours(result, contours4Plate, -1, Scalar(0),2);
            imshow("plate",result);
            isUnprotected = true;
            return true;
        }
    }
    return false;
}

void TrafficPlate::findSquares(Mat& image)
{
    this->squares.clear();
    int  N = 11;
    Mat pyr, timg, gray0(image.size(), CV_8U), gray;
    image.copyTo(timg);

    vector<vector<Point> > contours;
    for( int c = 0; c < 3; c++ ) //
    {
        int ch[] = {c, 0};
        mixChannels(&timg, 1, &gray0, 1, ch, 1);
        Mat dividedCh;
        gray0.copyTo(dividedCh);
       
        for( int l = 0; l < N; l++ )
        {
            if( l == 0 )
            {
                Canny(gray0, gray, 100,350, 3);
                dilate(gray, gray, Mat(), Point(-1,-1)); // remove hole
            }
            else
            {
                gray = gray0 >= (l+1)*255/N;
            }
            
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
                        double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                        maxCosine = MAX(maxCosine, cosine);
                    }
                    
                    if( maxCosine < 0.3 )
                    {
                        squares.push_back(approx);
                        Rect rect = boundingRect(Mat(approx));
                        Mat data = this->origin(Rect(rect.tl(),rect.br()));
                        Mat exp1;

                        resize(data, data, Size(sizeX,sizeY));
                        Mat grayRect(data.size(), CV_8U);
                        Mat edgeRect;
                        Mat grayRectrp;
                        
                        for( int c = 1; c < 3; c++ ) //
                        {
                            int ch[] = {c,0};
                            mixChannels(&data, 1, &grayRect, 1, ch, 1);
                            Mat dividedCh;
                            grayRect.copyTo(dividedCh);
                            
                            for( int l = 0; l < N; l++ )
                            {
                                
                                if( l == 0 )
                                {
                                    Canny(grayRect, edgeRect, 100,350, 3);
                                    dilate(edgeRect, edgeRect, Mat(), Point(-1,-1)); // remove hole
                                    edgeRect.copyTo(grayRectrp);
                                }
                                else
                                {
                                    grayRectrp = grayRect >= (l+1)*255/N;
                                    
                                }
                                
                                bool findPlate =  filterPlate(grayRectrp);
                                if(findPlate == true)
                                    findPlate;
                                
                            }
                            
                        }
                    }
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

//void TrafficPlate::featureDetect(Mat& image, vector<vector<Point>> squares)
//{
//    //-- Step 1: Detect the keypoints using SURF Detector
//    int minHessian = 400;
//    vector<KeyPoint> keypointsO; //keypoints for object
//    vector<KeyPoint> keypointsS; //keypoints for scene
//    Mat db_original = imread("/Users/sonhojun/Downloads/signals2.png");
//  //  equalizeHist(db_original, db_original);
//    Mat tmp;
//    
//    GaussianBlur(db_original, tmp, cv::Size(5,5), 5);
//    addWeighted(db_original, 1.5, tmp, -0.5, 0, db_original);
//    
//    
//    Mat descriptors_object, descriptors_scene;
//    
//   Ptr<xfeatures2d::SURF> surf = xfeatures2d::SURF::create(2500);
//    
//    surf->detect(db_original,keypointsO);
//    surf->compute( db_original, keypointsO, descriptors_object );
//    
//    FlannBasedMatcher matcher = FlannBasedMatcher();
//    //BFMatcher matcher = BFMatcher();
//    vector< DMatch > matches;
//    std::vector< std::vector<cv::DMatch> > os_matches, so_matches;
//    
//    for ( int i = 0; i< squares.size(); i++ )
//    {
//        Rect rect = boundingRect(Mat(squares[i]));
//
//        if((abs(rect.tl().x - rect.br().x)/abs(rect.tl().y - rect.br().y)) > 1.1)
//            continue;
//        cout<<"Rect x,y : " <<rect.tl().x <<" , " <<rect.tl().y<<endl;
//        Mat data = image(Rect(rect.tl(),rect.br()));
//        
//        
//        cvtColor(data, data, CV_BGR2GRAY);
//        Mat exp1;
//        resize(data, exp1, Size(db_original.cols,db_original.rows));
//       // pyrUp(data, exp1, Size(data.cols*2,data.rows*2));
//        
//        equalizeHist(exp1, exp1);
//        Mat tmp;
//        cv::GaussianBlur(exp1, tmp, cv::Size(5,5), 5);
//        cv::addWeighted(exp1, 1.5, tmp, -0.5, 0, exp1);
//        
//        threshold(exp1, exp1, 50, 255, THRESH_BINARY_INV);
//        imshow("cropsqu",exp1);
//        
//        surf->detect(exp1,keypointsS);
//        surf->compute(exp1, keypointsS, descriptors_scene );
//        
////        cout<<exp1.cols<<" , "<<exp1.rows<<endl;
////        cout<<descriptors_object.size << " , " << descriptors_scene.size << endl;
//        if(descriptors_scene.rows == 0 || descriptors_scene.rows == 1)
//            continue;
//        
//        
//        matcher.match(descriptors_object, descriptors_scene, matches);
////       matcher.knnMatch(descriptors_object, descriptors_scene, os_matches, 2);
//        
//        double max_dist = 0; double min_dist = 100;
//        
//        //-- Quick calculation of max and min distances between keypoints
//        for( int i = 0; i < descriptors_object.rows; i++ )
//        { double dist = matches[i].distance;
//            if( dist < min_dist ) min_dist = dist;
//            if( dist > max_dist ) max_dist = dist;
//        }
////        
//        printf("-- Max dist : %f \n", max_dist );
//        printf("-- Min dist : %f \n", min_dist );
////        
//        //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
//        //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
//        //-- small)
//        //-- PS.- radiusMatch can also be used here.
//        std::vector< DMatch > good_matches;
//        
//        for( int i = 0; i < descriptors_object.rows; i++ )
//        { if( matches[i].distance <= max(2*min_dist, 0.02) )
//        { good_matches.push_back( matches[i]); }
//        }
//        
//        //-- Draw only "good" matches
//        Mat img_matches;
//        if(keypointsS.size() == 0 )
//            continue;
//        drawMatches( db_original, keypointsO, exp1, keypointsS,
//                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//        
//        //-- Show detected matches
//        imshow( "Good Matches", img_matches );
////        
////        for( int i = 0; i < (int)good_matches.size(); i++ )
////        { printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }
////
//       
////        Mat img_matches;
////        drawMatches(db_original, keypointsO, exp1, keypointsS, os_matches, img_matches);
////        imshow("matches", img_matches);
////        
//     
//        
//        
//        //waitKey(0);
//    }
//}





