
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
    
    VideoCapture video("/Users/sonhojun/Downloads/data/driving1.mp4");
    
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
 
    UMat plateCrop;
    UMat colorRed;
    UMat colorGreen;
    
    vector<Vec3f> circles;
    vector<vector<Point>> squares;
    vector<vector<Point>> contours;
    vector<vector<Point>> newSquares;
    vector<vector<Point>> newCircles;
    vector<Point> heightwidth;
    
    cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
    //cv::Ptr<Feature2D> f2d = xfeatures2d::SURF::create();
    //cv::Ptr<Feature2D> f2d = ORB::create();
    
    namedWindow("test1");
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat db_original = imread("/Users/sonhojun/Downloads/signals2.png",CV_LOAD_IMAGE_GRAYSCALE);
    //pyrDown( db_original, db_original, Size( db_original.cols/2, db_original.rows/2 ));
    //pyrDown( db_original, db_original, Size( db_original.cols/2, db_original.rows/2 ));
    f2d->detect( db_original, keypoints_1 );
    Mat descriptors_1, descriptors_2;
    f2d->compute( db_original, keypoints_1, descriptors_1 );
    
    BFMatcher matcher;
    std::vector< DMatch > matches;
    
    DriverlessCar car = DriverlessCar();
    car.setState(0);
    
    while (1){
        video.read(image);
        video.read(image);

        //imshow("ori",image);
        /************** line detection ***************/
        if(1/*car.getState() == 0 || car.getState() == -1*/)
        {
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
            //        imshow( "white", White);
            //        imshow("yellow", Yellow);
            
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
            bool refinedPosi = car.finder.kmeansPositive();
            
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
            bool refinedNega = car.finder.kmeansNegative();
            
            
            /***** detection lane change ***********/
            int changedLane = car.laneChecker(refinedPosi, refinedNega);
            car.finder.drawLine(laneCrop);
            
            switch (changedLane) {
                case -1:
                    if(car.getWhichLane() != 1)
                        car.setWhichLane(car.getWhichLane() - 1);
                    break;
                case 0:
                    break;
                case 1:
                    if(car.getWhichLane() < 3)
                        car.setWhichLane(car.getWhichLane() + 1);
                    break;
                default:
                    break;
            }
            
            imshow("lines", laneCrop);
        }
        
        /***  detection front car *****/
        inRange(laneCrop,Scalar(25, 120, 140), Scalar(65, 255, 255),colorRed);
        car.findCircles(colorRed, circles);
        //imshow("red", colorRed);
        car.drawCircles(laneCrop, circles);
        if(circles.size() >= 1)
        {
            car.setState(-1);
            continue;
        }else
        {
            car.setState(1);
        }

        /*** detection plate and traffic lights *******/
        if(1/*car.getState() == 1*/) //horizontal detected
        {
            plateCrop = image(Rect(640,180,320,240));
            
            UMat temp;
            plateCrop.copyTo(temp);
            cvtColor(plateCrop, colorRed, CV_BGR2HSV);
            cvtColor(plateCrop, colorGreen, CV_BGR2HSV);
            
            inRange(colorRed,Scalar(169, 140, 150), Scalar(255, 255, 255),colorRed);
            //imshow("red2", colorRed);
            
            inRange(colorGreen,cv::Scalar(47, 140, 80), Scalar(94, 255, 180),colorGreen);
            // imshow("green",colorGreen);
            
            car.findCircles(colorRed, circles);
            if(circles.size() == 0)
                car.findCircles(colorGreen, circles);
            
            newSquares.clear();
            heightwidth.clear();
            
            car.findSquares(plateCrop, squares);
            
            for( size_t j = 0; j < squares.size(); j++ )
            {
                int maxX = 0;
                int minX = 1000;
                int maxY = 0;
                int minY = 1000;
                
                for(int maxi = 0 ; maxi< 4; maxi++)
                {
                    if(maxX < squares[j][maxi].x)
                    {
                        maxX = squares[j][maxi].x;
                    }
                    
                    if(minX > squares[j][maxi].x)
                    {
                        minX = squares[j][maxi].x;
                    }
                    
                    if(maxY < squares[j][maxi].y)
                    {
                        maxY = squares[j][maxi].y;
                    }
                    
                    if(minY > squares[j][maxi].y)
                    {
                        minY = squares[j][maxi].y;
                    }
                }
                heightwidth.push_back(Point(maxX - minX,maxY-minY));
            }
            
            //traffic lights 를 찾기 위함
            double trafficY = 0.;
            
            for( size_t i = 0; i < circles.size(); i++ )
            {
                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                for( size_t j = 0; j < squares.size(); j++ )
                {
                    int pX = cvRound((squares[j][0].x + squares[j][1].x + squares[j][2].x + squares[j][3].x) / 4);
                    int pY = cvRound((squares[j][0].y + squares[j][1].y + squares[j][2].y + squares[j][3].y) / 4);
                    
                    int maxX = 0;
                    int minX = 1000;
                    int maxY = 0;
                    int minY = 1000;
                
                    for(int maxi = 0 ; maxi< 4; maxi++)
                    {
                        if(maxX < squares[j][maxi].x)
                        {
                            maxX = squares[j][maxi].x;
                        }
                        if(minX > squares[j][maxi].x)
                        {
                            minX = squares[j][maxi].x;
                        }
                        if(maxY < squares[j][maxi].y)
                        {
                            maxY = squares[j][maxi].y;
                        }
                        if(minY > squares[j][maxi].y)
                        {
                            minY = squares[j][maxi].y;
                        }
                    }
                    heightwidth.push_back(Point(maxX - minX,maxY-minY));
                    // 세로가 가로보다 길면 제거
                    if((maxY-minY) > (maxX - minX)* 1.2)
                        continue;
                    
                    //cout<<"p: "<<p.x<<" , "<<p.y<<endl;
                    if(abs(pX - center.x) < 20 && abs(pY - center.y) < 4)
                    {
                        newSquares.push_back(squares.at(j));
                    }
                }
            }
            
            if(newSquares.size() > 0 )
            {
                car.drawTraffic(plateCrop,newSquares,circles);
                cout<<"find traffic"<<endl;
                //Todo traffic point 계산
                
            }
            
        if(heightwidth.size() == 0)
            continue;
            
        /**** detect unprotected left turn traffic signal *****/
        UMat sqaureCrop;
        int cols;
        int rows;
        
        cols = plateCrop.cols;
        rows = plateCrop.rows;
        
        for( size_t j = 0; j < squares.size(); j++ )
        {
            
            Point p = squares[j][0];
            
            int maxX = 0;
            int minX = 1000;
            int maxY = 0;
            int minY = 1000;
        
            for(int maxi = 0 ; maxi< 4; maxi++)
            {
                if(maxX < squares[j][maxi].x)
                {
                    maxX = squares[j][maxi].x;
                }
                if(minX > squares[j][maxi].x)
                {
                    minX = squares[j][maxi].x;
                }
                if(maxY < squares[j][maxi].y)
                {
                    maxY = squares[j][maxi].y;
                }
                if(minY > squares[j][maxi].y)
                {
                    minY = squares[j][maxi].y;
                }
            }
            
            cout<<"trafficY : " << trafficY << " , plate: " << (maxY - (maxY - minY)/2) << endl;
            // traffic lights랑 동시에 발견이 안되는 문제점.
//            if(abs(trafficY - (maxY - (maxY - minY)/2)) > 40.)
//            {
//                cout<<"not plate" << endl;
//                continue;
//            }
            
            if((maxY-minY) * 1.2 <(maxX - minX) || (maxY-minY) > (maxX - minX) * 1.4)
                continue;
            
            sqaureCrop = plateCrop(Rect(minX,minY,heightwidth[j].x ,heightwidth[j].y));
            //imshow("plate", sqaureCrop);
            UMat sqTemp;
            UMat whiteArrow;
            //pyrUp( sqaureCrop, sqaureCrop, Size( sqaureCrop.cols*2, sqaureCrop.rows*2 ) );
            //pyrUp( sqaureCrop, sqaureCrop, Size( sqaureCrop.cols*2, sqaureCrop.rows*2 ) );
            cvtColor(sqaureCrop, sqTemp, CV_BGR2HSV);
            
            inRange(sqTemp,cv::Scalar(90, 50, 50), Scalar(150, 255, 255),sqTemp); //blue?
            
            
            /** todo : traffic lights 가 있다면 그걸로 필터링 하기 ****/
            //todo : 가우시안 분산 에러 처리 찾아보기 ****/
           double totpix = sqTemp.cols*sqTemp.rows ;
            Mat tt;
            sqTemp.copyTo(tt);
            
            double tot=0.;
            
            for(int i=0; i<sqTemp.cols ; i++)
            {
                for( int j=0;j<sqTemp.rows; j++)
                {
                    if(tt.at<bool>(i, j) == 1)
                        tot++;
                }
            }
            //cout<<"RATIO: " << ((tot / totpix )*100) <<endl;
            if(((tot / totpix )*100)< 50 )
                continue;
            
            pyrUp( sqTemp, sqTemp, Size( sqTemp.cols*2, sqTemp.rows*2 ) );
            pyrUp( sqTemp, sqTemp, Size( sqTemp.cols*2, sqTemp.rows*2 ) );
            
            //imshow("test2",sqTemp);
            pyrUp( sqaureCrop, sqaureCrop, Size( sqaureCrop.cols*2, sqaureCrop.rows*2 ) );
            pyrUp( sqaureCrop, sqaureCrop, Size( sqaureCrop.cols*2, sqaureCrop.rows*2 ) );
            cvtColor(sqaureCrop, sqaureCrop, CV_BGR2GRAY);
            threshold(sqaureCrop, sqaureCrop,50,255,THRESH_BINARY);

            // addWeighted(sqaureCrop, 2.5, sqaureCrop, -1.5, 0, sqaureCrop);
           
            f2d->detect( sqaureCrop, keypoints_2 );
            f2d->compute( sqaureCrop, keypoints_2, descriptors_2 );
            
            matcher.match( descriptors_1, descriptors_2, matches );
            namedWindow("matches", 1);

            if(matches.size() == 0)
            {
                sqaureCrop.release();
                continue;
            }
            else
                cout<<"find plate : " << matches.size()<<endl;
            
            Mat img_matches;
            imshow("test1",sqaureCrop);
            drawMatches(db_original, keypoints_1, sqaureCrop, keypoints_2, matches, img_matches);
            imshow("matches", img_matches);
            
            sqaureCrop.release();
            
        }

    }

         
        cvWaitKey(20);
    }

    return 0;
}