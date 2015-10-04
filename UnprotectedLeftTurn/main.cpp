
#include "DriverlessCar.h"
#include "LineFinder.h"

using namespace std;
const char* wndname = "Square Detection Demo";

void MyFilledCircle( UMat& img, Point center )
{
    int thickness = 2;
    int lineType = 8;
    
    circle( img,
           center,
           200/32.0,
           Scalar(255,255,0),
           thickness,
           lineType );
    
}

int main()
{
    ocl::setUseOpenCL(false);
    
    VideoCapture video("/Users/sonhojun/Downloads/data/changeline2.mp4");
    
    if(!video.isOpened())
    {
        cout<<"video open error"<<endl;
        return 0 ;
    }
    
    UMat image;
    UMat laneCrop;
    UMat leftCrop;
    UMat rightCrop;
    
    UMat White;
    UMat WhiteAppendix;
    UMat Yellow;
    UMat bw;
    UMat edgeWhite;
    UMat edgeYellow;
    UMat edgeTotal;
    UMat element(3,3,CV_8U,Scalar(1));
    UMat tpElement(1,1,CV_8U,Scalar(255));
 
    DriverlessCar car = DriverlessCar();
    car.setState(0);
    
    while (1){
        car.setState(1);
        video.read(image);
        video.read(image);
       
        /************** line detection ***************/
        laneCrop = image(Rect(LANECROP_X,LANECROP_Y,LANECROP_W,LANECROP_H));
        
        car.setCenterX(laneCrop.cols/2);
        car.setCenterY(laneCrop.rows);
        
        //inRange(laneCrop,cv::Scalar( 0, 0, 210), cv::Scalar(255, 255, 255),White);
        //imshow("rgb", White);
        
        if(image.channels() == 3)
            cvtColor(laneCrop, laneCrop, CV_BGR2HSV);

        inRange(laneCrop,cv::Scalar( 0, 0, 235), cv::Scalar(255, 255, 255),White);
        //inRange(laneCrop,cv::Scalar( 0, 0, 180), cv::Scalar(10, 70, 255),WhiteAppendix);
        inRange(laneCrop,cv::Scalar( 10,10,165), cv::Scalar(65,250,255),Yellow);

//      morphologyEx(White, White, MORPH_ERODE, element);
//      morphologyEx(Yellow, Yellow, MORPH_ERODE, element);
//      bitwise_or(White, WhiteAppendix, White);
        imshow( "white", White);
        imshow("yellow", Yellow);

        bitwise_or(White, Yellow, bw);
        //morphologyEx(bw, bw, MORPH_ERODE, element);
        //imshow("bw",bw);
        
        MyFilledCircle(laneCrop, Point(car.getCenterX(),car.getCenterY()));
        MyFilledCircle(laneCrop, Point(car.getEndX(),car.getEndY()));
        
        /*** detection yellow line ****/
        Canny(Yellow,edgeYellow, 100,350,5);
        car.finder.setLineLengthAndGap(50,30);  //line pixel and gap
        car.finder.setMinVote(40);
        car.finder.findLines(edgeYellow, 0);
        car.finder.filterLine(edgeYellow,car.getCenterX(),car.getCenterY(), 0);
        car.finder.kmeansPositive();
        
        if(car.finder.getPositiveLines().size() >= 1)
        {
            car.setWhichLane(1);
        }

        /*** detection white horizontal line ****/
        Canny(bw,edgeWhite, 100,350,5);
        car.finder.setLineLengthAndGap(100,10);  //line pixel and gap
        car.finder.setMinVote(60);
        car.finder.findLines(edgeWhite, 2);
        car.finder.filterLine(edgeWhite,car.getCenterX(),car.getCenterY(), 2);
        
        car.finder.filterStopLine();
        
        if(car.finder.getHorizontalLines().size() >= 1)
        {
            cout<<"possible stop line"<< endl;
            car.setState(1);
            
        }
        //car.finder.drawLine(laneCrop);
        
        /*** detection white negative line ****/
        Canny(bw,edgeWhite, 100,350,5);
        car.finder.setLineLengthAndGap(50,30);  //line pixel and gap
        car.finder.setMinVote(40);
        car.finder.findLines(edgeWhite, 1);
        car.finder.filterLine(edgeWhite,car.getCenterX(),car.getCenterY(), 1);
        car.finder.kmeansNegative();
        
        car.laneChecker();
        car.finder.drawLine(laneCrop);

        imshow("lines", laneCrop);
        
        /**** detection line change *****/
        
        
        
        cvWaitKey(20);
    }

    return 0;
}