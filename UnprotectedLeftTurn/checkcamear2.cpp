#include "MetaHeader.h"
#include <stdio.h>
using namespace cv;
using namespace std;
const static int SENSITIVITY_VALUE = 30;
const static int BLUR_SIZE = 15;
#define W 640
#define H 480

int BlockSize_MIN = 3;
int BlockSize_MAX = 128;
int NumDisparities_MIN = 1;
int NumDisparities_MAX = 16;
int PreFilterSize_MIN = 3;
int PreFilterSize_MAX = 128;
int PreFilterCap_MIN = 1;
int PreFilterCap_MAX = 63;
int MinDisparity_MIN = 0;
int MinDisparity_MAX = 100;
int TextureThreshold_MIN = 0;
int TextureThreshold_MAX = 1000;
int UniquenessRatio_MIN = 0;
int UniquenessRatio_MAX = 10;
int SpeckleWindowSize_MIN = 5;
int SpeckleWindowSize_MAX = 255;
int SpeckleRange_MIN = 5;
int SpeckleRange_MAX = 255;
static Point mousePos;
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if (event == EVENT_RBUTTONDOWN)
    {
        cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if (event == EVENT_MBUTTONDOWN)
    {
        cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if (event == EVENT_MOUSEMOVE)
    {
        cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
        mousePos.x = x; mousePos.y = y;
    }
}

const string trackbarWindowName = "Trackbars";
void on_trackbar(int, void*)
{//This function gets called whenever a
    // trackbar position is changed
}
void createTrackbars() {
    //create window for trackbars
    namedWindow(trackbarWindowName, 0);
    //create memory to store trackbar name on window
    char TrackbarName[50];
    sprintf(TrackbarName, "BlockSize_MIN", BlockSize_MIN);
    sprintf(TrackbarName, "BlockSize_MAX", BlockSize_MAX);
    sprintf(TrackbarName, "NumDisparities_MIN", NumDisparities_MIN);
    sprintf(TrackbarName, "NumDisparities_MAX", NumDisparities_MAX);
    sprintf(TrackbarName, "PreFilterSize_MIN", PreFilterSize_MIN);
    sprintf(TrackbarName, "PreFilterSize_MAX", PreFilterSize_MAX);
    sprintf(TrackbarName, "PreFilterCap_MIN", PreFilterCap_MIN);
    sprintf(TrackbarName, "PreFilterCap_MAX", PreFilterCap_MAX);
    sprintf(TrackbarName, "MinDisparities_MIN", MinDisparity_MIN);
    sprintf(TrackbarName, "MinDisparities_MAX", MinDisparity_MAX);
    sprintf(TrackbarName, "TextureThreshold_MIN", TextureThreshold_MIN);
    sprintf(TrackbarName, "TextureThreshold_MAX", TextureThreshold_MAX);
    sprintf(TrackbarName, "UniquenessRatio_MIN", UniquenessRatio_MIN);
    sprintf(TrackbarName, "UniquenessRatio_MAX", UniquenessRatio_MAX);
    sprintf(TrackbarName, "SpeckleWindowSize_MIN", SpeckleWindowSize_MIN);
    sprintf(TrackbarName, "SpeckleWindowSize_MAX", SpeckleWindowSize_MAX);
    sprintf(TrackbarName, "SpeckleRange_MIN", SpeckleRange_MIN);
    sprintf(TrackbarName, "SpeckleRange_MAX", SpeckleRange_MAX);
    //create trackbars and insert them into window
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->
    createTrackbar("BlockSize", trackbarWindowName, &BlockSize_MIN, BlockSize_MAX, on_trackbar);
    //createTrackbar("BlockSize_MAX", trackbarWindowName, &BlockSize_MAX, BlockSize_MAX, on_trackbar);
    createTrackbar("NumDisparities", trackbarWindowName, &NumDisparities_MIN, NumDisparities_MAX, on_trackbar);
    //createTrackbar("NumDisparities_MAX", trackbarWindowName, &NumDisparities_MAX, NumDisparities_MAX, on_trackbar);
    createTrackbar("PreFilterSize", trackbarWindowName, &PreFilterSize_MIN, PreFilterSize_MAX, on_trackbar);
    //createTrackbar("PreFilterSize_MAX", trackbarWindowName, &PreFilterSize_MAX, PreFilterSize_MAX, on_trackbar);
    createTrackbar("PreFilterCap", trackbarWindowName, &PreFilterCap_MIN, PreFilterCap_MAX, on_trackbar);
    //createTrackbar("PreFilterCap_MAX", trackbarWindowName, &PreFilterCap_MAX, PreFilterCap_MAX, on_trackbar);
    createTrackbar("MinDisparity", trackbarWindowName, &MinDisparity_MIN, MinDisparity_MAX, on_trackbar);
    //createTrackbar("MinDisparities_MAX", trackbarWindowName, &MinDisparity_MAX, MinDisparity_MAX, on_trackbar);
    createTrackbar("TextureThreshold", trackbarWindowName, &TextureThreshold_MIN, TextureThreshold_MAX, on_trackbar);
    //createTrackbar("TextureThreshold_MAX", trackbarWindowName, &TextureThreshold_MAX, TextureThreshold_MAX, on_trackbar);
    createTrackbar("UniquenessRatio", trackbarWindowName, &UniquenessRatio_MIN, UniquenessRatio_MAX, on_trackbar);
    //createTrackbar("UniquenessRatio_MAX", trackbarWindowName, &UniquenessRatio_MAX, UniquenessRatio_MAX, on_trackbar);
    createTrackbar("SpeckleWindowSize", trackbarWindowName, &SpeckleWindowSize_MIN, SpeckleWindowSize_MAX, on_trackbar);
    //createTrackbar("SpeckleWindowSize_MAX", trackbarWindowName, &SpeckleWindowSize_MAX, SpeckleWindowSize_MAX, on_trackbar);
    createTrackbar("SpeckleRange", trackbarWindowName, &SpeckleRange_MIN, SpeckleRange_MAX, on_trackbar);
    //createTrackbar("SpeckleRange_MAX", trackbarWindowName, &SpeckleRange_MAX, SpeckleRange_MAX, on_trackbar);
}
int main(int argc, char* argv[])
{
    int numBoards = 50;
    int board_w = 23;
    int board_h = 18;
    Size board_sz = Size(board_w, board_h);
    int board_n = board_w*board_h;
    vector<vector<Point3f> > object_points;
    vector<vector<Point2f> > imagePoints1, imagePoints2;
    vector<Point2f> corners1, corners2;
    vector<Point3f> obj;
    createTrackbars();
    for (int j = 0; j<board_n; j++)
    {
        obj.push_back(Point3f(j / board_w, j%board_w, 4.3f));
    }
    Mat img1, img2, gray1, gray2;
    VideoCapture cap1 = VideoCapture(0);
    VideoCapture cap2 = VideoCapture(2);
    int success = 0, k = 0;
    bool found1 = false, found2 = false;
    namedWindow("disparity", 1);
    setMouseCallback("disparity",CallBackFunc,NULL);
    cap1 >> img1;
    cap2 >> img2;
    resize(img1, img1, Size(W, H), INTER_LINEAR);
    resize(img2, img2, Size(W, H), INTER_LINEAR);
    
    double mCM1[] = { 449.7813675387458, 0, 331.4100142850414,
        0, 617.1872803490929, 256.7785765281859,
        0, 0, 1 };
    Mat CM1(3, 3, CV_64F, mCM1);
    double mCM2[] = { 449.7813675387458, 0, 311.3231326831822,
        0, 617.1872803490929, 238.4285688209055,
        0, 0, 1 };
    Mat CM2(3, 3, CV_64F, mCM2);
    float mD1[] = { 0.01013053354495019, -0.01559818871378516, 0, 0, -0.1063938503860138 };
    Mat D1(1, 4, CV_32F, mD1);
    float mD2[] = { 0.007186654700762747, 0.0749500266942328, 0, 0, -0.2917473484844327 };
    Mat D2(1, 4, CV_32F, mD2);
    float mR[] = { 0.998519827846145, -0.01334657730070809, 0.05272591651608539,
        0.01255215050466894, 0.9998030895415519, 0.01536963437676643,
        -0.05292066624498526, -0.01468506103234048, 0.9984907370960751 };
    Mat R(3, 3, CV_64F, mR);
    float mT[] = { 11.15776484624414,
        0.1156210134963671,
        -0.3385249256415286 };
    Mat T(3, 1, CV_64F, mT);
    float mE[] = { -0.001869525249913851, 0.3367603649034093, 0.1206495153243152,
        0.2524524989949072, 0.1683706068398895, -11.15877388263898,
        0.02460406915397147, 11.15711091044723, 0.1653945422435953 };
    Mat E(3, 3, CV_64F, mE);
    float mF[] = { 1.276842196093997e-07, -1.67614411377645e-05, 0.0005554283335211516,
        -1.256521890038447e-05, -6.107186118740606e-06, 0.2555415730144702,
        0.002200343224013052, -0.2430975584609801, 1 };
    Mat F(3, 3, CV_64F, mF);
    
    //    Mat CM1 = Mat::eye(3, 3, CV_64F);
    //    Mat CM2 = Mat::eye(3, 3, CV_64F);
    //    Mat D1, D2; //k1,k2,p1,p1 4*1
    //
    //Mat R, T, E, F;
    //cout<<"size: " <<img1.size()<<endl;
    //
    //double rms = stereoCalibrate(object_points, imagePoints1, imagePoints2,
    //                             CM1, D1, CM2, D2, img1.size(), R, T, E, F,
    //                             CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST| CV_CALIB_USE_INTRINSIC_GUESS /*| CV_CALIB_FIX_INTRINSIC*/, cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5)
    //                             );
    //
    //cout<<"Error: " <<rms<<endl;
    //
    //        FileStorage fs1("mystereocalib.yml", FileStorage::WRITE);
    //        fs1 << "CM1" << CM1;
    //        fs1 << "CM2" << CM2;
    //        fs1 << "D1" << D1;
    //        fs1 << "D2" << D2;
    //        fs1 << "R" << R;
    //        fs1 << "T" << T;
    //        fs1 << "E" << E;
    //        fs1 << "F" << F;
    cout << "CM1: " << CM1 << endl;
    cout << "CM2: " << CM2 << endl;
    cout << "D1: " << D1 << endl;
    cout << "D2: " << D2 << endl;
    cout << "R: " << R << endl;
    cout << "T: " << T << endl;
    cout << "E: " << E << endl;
    cout << "F:" << F << endl;
    cout << "Done Calibration\n" << endl;
    cout << "Starting Rectification\n" << endl;
    Mat R1, R2, P1, P2, Q;
    stereoRectify(CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);
    
    cout << "R1: " << R1 << endl;
    cout << "R2: " << R2 << endl;
    cout << "P1: " << P1 << endl;
    cout << "P2: " << P2 << endl;
    cout << "Q: " << Q << endl;
    cout << "Done Rectification\n" << endl;
    cout << "Applying Undistort\n" << endl;
    Mat map1x, map1y, map2x, map2y;
    Mat imgU1, imgU2;
    initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CV_16SC2, map1x, map1y);
    initUndistortRectifyMap(CM2, D2, R2, P2, img2.size(), CV_16SC2, map2x, map2y);
    cout << "Undistort complete\n" << endl;
    //    VideoCapture vCap1("/Users/sonhojun/Downloads/data/driving3.mp4");
    //    VideoCapture vCap2("/Users/sonhojun/Downloads/data/driving3.mp4");
    Mat frame1rf;
    Mat frame2rf;
    
    
    
    
    while (1)
    {
        cap1 >> img1;
        cap2 >> img2;
        resize(img1, img1, Size(W, H), INTER_LINEAR);
        resize(img2, img2, Size(W, H), INTER_LINEAR);
        remap(img1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
        remap(img2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
        cvtColor(img1, imgU1, CV_RGB2GRAY);
        cvtColor(img2, imgU2, CV_RGB2GRAY);
        imshow("image1", imgU1);
        imshow("image2", imgU2);
        
        Ptr<StereoBM> sbm = StereoBM::create();
        sbm->setBlockSize(2*BlockSize_MIN-1);//홀수
        sbm->setNumDisparities(16*NumDisparities_MIN);//16배수
        sbm->setPreFilterSize(2*PreFilterSize_MIN-1);//홀수
        sbm->setPreFilterCap(PreFilterCap_MIN);
        sbm->setMinDisparity(MinDisparity_MIN-50);//-50~50
        sbm->setTextureThreshold(TextureThreshold_MIN);// max  1000
        sbm->setUniquenessRatio(UniquenessRatio_MIN);
        sbm->setSpeckleWindowSize(0);
        sbm->setSpeckleRange(8);
        
//        sbm->setBlockSize(2*53-1);//홀수
//        sbm->setNumDisparities(16*10);//16배수
//        sbm->setPreFilterSize(2*3-1);//홀수
//        sbm->setPreFilterCap(1);
//        sbm->setMinDisparity(40-50);//-50~50
//        sbm->setTextureThreshold(201);// max  1000
//        sbm->setUniquenessRatio(0);
//        sbm->setSpeckleWindowSize(0);
//        sbm->setSpeckleRange(8);
    //sbm->setDisp12MaxDiff(1);
        
        Mat disp(imgU2.size(), CV_32F);
        Mat disp8(imgU2.size(), CV_8U);
        sbm->compute(imgU2, imgU1, disp);
        double minVal; double maxVal;
        minMaxLoc(disp, &minVal, &maxVal);
        // printf("Min disp: %f Max value: %f \n", minVal, maxVal);
        disp.convertTo(disp8, CV_8UC1, 255 / (maxVal - minVal));
        //bitwise_and(thresholdImage, disp8, disp8);
        //특정 값보다 큰 값만 찾아내기
        //Threshold
        
        imshow("disparity", disp8);
        cout << "pos: " << mousePos.x << ", " << mousePos.y << endl;
        unsigned int tt= disp8.at<uchar>(mousePos.y, mousePos.x);
        cout << "value:" << tt << endl;
        cvWaitKey(10);
        
//        Mat rectified_pair;
//        rectified_pair.create(imgU2.rows, (imgU2.cols)*2, CV_8UC3);
//        
//        cvtColor(imgU2, frame1rf, CV_GRAY2BGR);
//        frame1rf.copyTo(rectified_pair(Rect(0,0,imgU2.cols, imgU2.rows)));
//        //copy frame2r to the right side
//        cvtColor(imgU1, frame2rf, CV_GRAY2BGR);
//        frame2rf.copyTo(rectified_pair(Rect(imgU2.cols,0,imgU2.cols, imgU2.rows)));
//        for(int i=0; i<imgU2.rows; i+=32)
//            line(rectified_pair, Point(0,i), Point((imgU2.cols)*2, i), CV_RGB(0,255,0));
//        imshow("rectified", rectified_pair);
        cvWaitKey(10);
        sbm.release();
        disp.release();
        disp8.release();
        imgU1.release();
        img2.release();
        cvWaitKey(100);
    }
    cap1.release();
    cap2.release();
    return(0);
}