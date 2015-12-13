//
//  checkCamera.cpp
//  UnprotectedLeftTurn
//
//  Created by SonHojun on 12/4/15.
//  Copyright © 2015 SonHojun. All rights reserved.
//
#include "MetaHeader.h"


int main()
{
    Mat img1, img2, gray1, gray2;
    VideoCapture cap1 = VideoCapture(0);
    VideoCapture cap2 = VideoCapture(2);
    int success = 0, k = 0;
    bool found1 = false, found2 = false;
    
    while(1)
    {
        cap1 >> img1;
        cap2 >> img2;
        
        imshow("camera1", img1);
        imshow("camera2", img2);
    }
    
    return 0;
}