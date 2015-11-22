#include "MetaHeader.h"
#include "UV.h"
#include <future>

int window_size = 9;
int temp1;
int number_of_disparities = 80;
int temp2;
int pre_filter_size = 5;
int temp3;
int pre_filter_cap = 23;
int temp4;
int min_disparity = 30;
int temp5;
int texture_threshold = 500;
int temp6;
int uniqueness_ratio = 0;
int temp7;
int max_diff = 100;
float temp8;
int speckle_window_size = 0;
int temp9;

int main()
{
 //   ocl::setUseOpenCL(true);
    
    VideoCapture videoLeft(0);
    videoLeft.set(CV_CAP_PROP_FRAME_WIDTH, 720); //added
    videoLeft.set(CV_CAP_PROP_FRAME_HEIGHT, 640); //added
    VideoCapture videoRight(2);
    videoRight.set(CV_CAP_PROP_FRAME_WIDTH, 720); //added
    videoRight.set(CV_CAP_PROP_FRAME_HEIGHT, 640); //added

    if(!videoLeft.isOpened())
    {
        cout<<"video open error"<<endl;
        return 0 ;
    }
    
    Mat temp;
    Mat right;
    Mat disparity;
    

//    Mat imgR = imread("/Users/sonhojun/Downloads/Yeuna9x.png");
//    Mat imgL = imread("/Users/sonhojun/Downloads/SuXT483.png");
//
//    cvtColor(imgR, imgR, CV_RGB2GRAY);
//    cvtColor(imgL, imgL, CV_RGB2GRAY);
//    
//    Mat imgDisparity16S = Mat( imgR.rows, imgR.cols, CV_16S );
//    //
//    Mat imgDisparity8U = Mat( imgR.rows, imgR.cols, CV_8UC1 );
//
//    Ptr<StereoBM> sbm= StereoBM::create(16, 15);
//    sbm->compute(imgL, imgR, imgDisparity16S);
//    
//    double minVal; double maxVal;
//    minMaxLoc( imgDisparity16S, &minVal, &maxVal );
//    
//    //printf("Min disp: %f Max value: %f \n", minVal, maxVal);
//    imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));
//    
//    imshow("dis",imgDisparity8U);
//
//    waitKey();

    while (1)
    {
        
        videoLeft.read(temp);
        videoRight.read(right);
//        cvtColor(temp, temp, CV_RGB2GRAY);
//        cvtColor(right, right, CV_RGB2GRAY);

    
//        vector<Point2f> selPoints1, selPoints2;
//        vector<KeyPoint> keypoints1, keypoints2;
//        
//        Ptr<Feature2D> fast = FastFeatureDetector::create(20);
//        fast->detect(temp, keypoints1);
//        
//        //drawKeypoints(temp,keypoints1, temp, Scalar(255),DrawMatchesFlags::DRAW_OVER_OUTIMG);
//        //imshow("temp",temp);
//        
//        
//        fast->detect(right, keypoints2);
//        
//        //drawKeypoints(right,keypoints2, right, Scalar(255),DrawMatchesFlags::DRAW_OVER_OUTIMG);
//        //imshow("right",right);
//        
//
//        KeyPoint::convert(keypoints1, selPoints1);
//        KeyPoint::convert(keypoints2, selPoints2);
//        
//        Mat fundemental = findFundamentalMat(Mat(selPoints1), Mat(selPoints2),CV_FM_7POINT);
//        
//        vector<Vec3f> lines1;
//        computeCorrespondEpilines(Mat(selPoints1), 1, fundemental, lines1);
//        
//        for(vector<Vec3f>::const_iterator it = lines1.begin(); it!=lines1.end(); ++it)
//        {
//            line(right, Point(0, -(*it)[2]/(*it)[1]), Point(right.cols,-((*it)[2]+(*it)[0]*right.cols)/(*it)[1]),
//                 Scalar(255,255,255));
//        }
//        
//        imshow("right", right);
//        
        
        imshow("LEFT",temp);
        imshow("RIGHT",right);
        
        cvtColor(temp, temp, CV_RGB2GRAY);
        cvtColor(right, right, CV_RGB2GRAY);
        
        Mat imgDisparity16S = Mat( temp.rows, temp.cols, CV_16S );
            //
        Mat imgDisparity8U = Mat( temp.rows, temp.cols, CV_8UC1 );
        
        Ptr<StereoBM> sbm= StereoBM::create(64, 19);
        sbm->compute(temp, right, imgDisparity16S);
        
        double minVal; double maxVal;
        minMaxLoc( imgDisparity16S, &minVal, &maxVal );
        
            //printf("Min disp: %f Max value: %f \n", minVal, maxVal);
        imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));
        
        imshow("dis",imgDisparity8U);

        
//        findFundamentalMat(temp, right, disparity);
//        
//        imshow("out", disparity);
//        
//        Mat imgDisparity16S = Mat( temp.rows, temp.cols, CV_32S );
//        
//        Mat imgDisparity8U = Mat( right.rows, right.cols, CV_8UC1 );
//        
//        
////        
////        Ptr<StereoBM> sbm = cv::StereoBM::create(16,15);
////        sbm->compute(temp,right,disparity);
////        imshow("disparity",disparity);
////        
//        
//        int ndisparities = 16*5;
//        int SADWindowSize = 60;
//        Ptr<StereoBM> sbm = StereoBM::create( 64, 19 );
//
//        //-- 3. Calculate the disparity image
//        sbm->compute( temp, right, imgDisparity16S );
//        //imshow("dis",imgDisparity16S);
//        double minVal; double maxVal;
//        minMaxLoc( imgDisparity16S, &minVal, &maxVal );
//
//        printf("Min disp: %f Max value: %f \n", minVal, maxVal);
//        imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));
//        
//        imshow( "disparity", imgDisparity8U );
        
        cvWaitKey(10);
    }
    
    return 0;
}