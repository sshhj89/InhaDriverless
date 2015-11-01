//
//  Header.h
//  UnprotectedLeftTurn
//
//  Created by SonHojun on 10/15/15.
//  Copyright (c) 2015 SonHojun. All rights reserved.
//

#ifndef __HEADER_H__
#define __HEADER_H__

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/ml/ml.hpp>
#include <opencv2/hal.hpp>

#include <opencv2/core/ocl.hpp>
#include <cmath>

#include <iostream>
#include <cstdlib>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/tracking.hpp>

#define PLATEDETECTED 3
#define PLATECROP_X 640
#define PLATECROP_Y 180
#define PLATECROP_W 180
#define PLATECROP_H 200

#define LANECROP_X 500
#define LANECROP_Y 460
#define LANECROP_W 360
#define LANECROP_H 180

#define WHITE_HUE_MIN 0
#define WHITE_SAT_MIN 0
#define WHITE_INT_MIN 235

#define WHITE_HUE_MAX 255
#define WHITE_SAT_MAX 255
#define WHITE_INT_MAX 255

#define YELLOW_HUE_MIN 10
#define YELLOW_SAT_MIN 10
#define YELLOW_INT_MIN 165

#define YELLOW_HUE_MAX 65
#define YELLOW_SAT_MAX 250
#define YELLOW_INT_MAX 255

#define RED_HUE_MIN 2
#define RED_SAT_MIN 0
#define RED_INT_MIN 240

#define RED_HUE_MAX 40
#define RED_SAT_MAX 255
#define RED_INT_MAX 255

#define GREEN_HUE_MIN 0
#define GREEN_SAT_MIN 0
#define GREEN_INT_MIN 240

#define GREEN_HUE_MAX 40
#define GREEN_SAT_MAX 255
#define GREEN_INT_MAX 255

#define LIGHTS_HUE_MIN 0
#define LIGHTS_SAT_MIN 0
#define LIGHTS_INT_MIN 200

#define LIGHTS_HUE_MAX 40
#define LIGHTS_SAT_MAX 255
#define LIGHTS_INT_MAX 255


#define SQUARE_HUE_MIN 0
#define SQUARE_SAT_MIN 0
#define SQUARE_INT_MIN 0

#define SQUARE_HUE_MAX 80
#define SQUARE_SAT_MAX 255
#define SQUARE_INT_MAX 255


using namespace std;
using namespace cv;


#endif
