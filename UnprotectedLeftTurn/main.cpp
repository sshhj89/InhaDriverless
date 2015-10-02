
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
    
    VideoCapture video("/Users/sonhojun/Downloads/data/traffic1.mp4");
    
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
    
/*    UMat plateCrop;
    UMat grayPlateCrop;
    UMat green;
    UMat red;
*/
    
    DriverlessCar car = DriverlessCar();

/*  vector<vector<Point> > squares;
    vector<vector<Point> > contours;
    vector<Vec3f> circles;
    
    vector<vector<Point> > newSquares;
    vector<vector<Point> > newCircles;
    vector<Point> heightwidth;
    
    cv::Ptr<Feature2D> f2d = xfeatures2d::SIFT::create();
    //cv::Ptr<Feature2D> f2d = xfeatures2d::SURF::create();
    //cv::Ptr<Feature2D> f2d = ORB::create();
    // you get the picture, i hope..
    
    namedWindow("test1");
    //-- Step 1: Detect the keypoints:
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat db_original = imread("/Users/sonhojun/Downloads/signals2.png",CV_LOAD_IMAGE_GRAYSCALE);
    
    //pyrDown( db_original, db_original, Size( db_original.cols/2, db_original.rows/2 ));
    //pyrDown( db_original, db_original, Size( db_original.cols/2, db_original.rows/2 ));
    
    f2d->detect( db_original, keypoints_1 );
    //-- Step 2: Calculate descriptors (feature vectors)
    Mat descriptors_1, descriptors_2;
    f2d->compute( db_original, keypoints_1, descriptors_1 );
    
    BFMatcher matcher;
    std::vector< DMatch > matches;
*/
    while (1){
        car.setState(1);
        video.read(image);
        video.read(image);
       
        /************** line detection ***************/
        laneCrop = image(Rect(LANECROP_X,LANECROP_Y,LANECROP_W,LANECROP_H));
        
        car.setCenterX(laneCrop.cols/2);
        car.setCenterY(laneCrop.rows);
        
        if(image.channels() == 3)
            cvtColor(laneCrop, laneCrop, CV_BGR2HSV);

        inRange(laneCrop,cv::Scalar( 0, 0, 235), cv::Scalar(255, 255, 255),White);
        //inRange(laneCrop,cv::Scalar( 0, 0, 180), cv::Scalar(10, 70, 255),WhiteAppendix);
        inRange(laneCrop,cv::Scalar( 10,10,165), cv::Scalar(65,250,255),Yellow);

//      morphologyEx(White, White, MORPH_ERODE, element);
//      morphologyEx(Yellow, Yellow, MORPH_ERODE, element);
//      bitwise_or(White, WhiteAppendix, White);
//      imshow( "white", White);
//      imshow("yellow", Yellow);

        bitwise_or(White, Yellow, bw);
        //morphologyEx(bw, bw, MORPH_ERODE, element);
        //imshow("bw",bw);
        
        
        /*** detection yellow line ****/

        Canny(Yellow,edgeYellow, 100,350,5);
        car.finder.setLineLengthAndGap(50,30);  //line pixel and gap
        car.finder.setMinVote(40);
        car.finder.findLines(edgeYellow);
        car.finder.filterLine(edgeYellow,car.getCenterX(),car.getCenterY(), 3);
        car.finder.kmeansPositive();
        
        if(car.finder.getPositiveLines().size() >= 1)
        {
            cout<<"1 lane"<< endl;
            car.setWhichLane(1);
            car.setState(0);
        }
        car.finder.filterStopLine();
        
        car.finder.drawLine(laneCrop);
        MyFilledCircle(laneCrop, Point(car.getCenterX(),car.getCenterY()));
        MyFilledCircle(laneCrop, Point(car.getEndX(),car.getEndY()));
        imshow("lines", laneCrop);
        
        vector<Vec4i>::const_iterator it = car.finder.getPositiveLines().begin();
        
        while(it != car.finder.getPositiveLines().end())
        {
            Point pt1((*it)[0],(*it)[1]);
            Point pt2((*it)[2],(*it)[3]);
            cout<<car.getCriteriaLine().ArcCalc(car.getCenterX() - car.getEndX(), car.getCenterY()-car.getEndY(),
                                                (*it)[0] - (*it)[2], (*it)[1] - (*it)[3])<<endl;
            ++it;
        }
        
        
//         car.finder.kmeansNegative();
/*
        if(car.finder.getNagativeLines().size() != 0)
            car.setState(0);
        

        plateCrop = image(Rect(640,180,320,240));
        UMat temp;
        plateCrop.copyTo(temp);
 
        
        UMat colorRed;
        UMat colorGreen;
        cvtColor(plateCrop, colorRed, CV_BGR2HSV);
        cvtColor(plateCrop, colorGreen, CV_BGR2HSV);
        
        //imshow("red1", colorRed);
        inRange(colorRed,Scalar(169, 110, 150), Scalar(174, 255, 250),colorRed);
        //imshow("red", colorRed);
        
        
        
       // imshow("green1",colorGreen);
        inRange(colorGreen,cv::Scalar(47, 140, 80), Scalar(94, 255, 180),colorGreen);
       // imshow("green",colorGreen);

       car.findCircles(colorGreen, circles);
       //car.drawCircles(temp, circles);
        
       newSquares.clear();
       heightwidth.clear();
        
        car.findSquares(temp, squares);
        //unprotected left turn signal 찾기 위함
        //car.drawSquares(temp, squares);
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
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            //cout<<center.x<<", " <<center.y<<endl;
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
            //car.drawSquares(temp, newSquares);
            //cvWaitKey(1000);
            cout<<"iinnnnnnnnn"<<endl;
        }

        car.drawSquares(temp, squares);
        
        if(heightwidth.size() == 0)
            continue;
        
        UMat sqaureCrop;
        int cols;
        int rows;
        
        cols = plateCrop.cols;
        rows = plateCrop.rows;
        
        for( size_t j = 0; j < squares.size(); j++ )
        {
            
            Point p = squares[j][0];
            cout<<"p: "<<p.x<<" , "<<p.y<<endl;
            cout<<squares[j][1].x << ", " << squares[j][1].y<<endl;
            cout<<squares[j][2].x << ", " << squares[j][2].y<<endl;
            cout<<squares[j][3].x << ", " << squares[j][3].y<<endl;
            cout<<"cols: " << cols << " rows : " << rows<<endl;
            cout<<"hiehgt: " << heightwidth[j].x << ", " << heightwidth[j].y<<endl;
            
            
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
            
            if((maxY-minY) * 1.2 <(maxX - minX) || (maxY-minY) > (maxX - minX) * 1.4)
                continue;

            sqaureCrop = plateCrop(Rect(minX,minY,heightwidth[j].x ,heightwidth[j].y));
            UMat sqTemp;
            UMat whiteArrow;
//            pyrUp( sqaureCrop, sqaureCrop, Size( sqaureCrop.cols*2, sqaureCrop.rows*2 ) );
//            pyrUp( sqaureCrop, sqaureCrop, Size( sqaureCrop.cols*2, sqaureCrop.rows*2 ) );
            cvtColor(sqaureCrop, sqTemp, CV_BGR2HSV);
            
            inRange(sqTemp,cv::Scalar(90, 50, 50), Scalar(150, 255, 255),sqTemp);
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

            cvWaitKey(100);
            
            cout<<"RATIO: " << ((tot / totpix )*100) <<endl;
            if(((tot / totpix )*100)< 50 )
                continue;
            
            pyrUp( sqTemp, sqTemp, Size( sqTemp.cols*2, sqTemp.rows*2 ) );
            pyrUp( sqTemp, sqTemp, Size( sqTemp.cols*2, sqTemp.rows*2 ) );
            
            imshow("test2",sqTemp);
            pyrUp( sqaureCrop, sqaureCrop, Size( sqaureCrop.cols*2, sqaureCrop.rows*2 ) );
            pyrUp( sqaureCrop, sqaureCrop, Size( sqaureCrop.cols*2, sqaureCrop.rows*2 ) );
            cvtColor(sqaureCrop, sqaureCrop, CV_BGR2GRAY);
            threshold(sqaureCrop, sqaureCrop,50,255,THRESH_BINARY);
            
           // addWeighted(sqaureCrop, 2.5, sqaureCrop, -1.5, 0, sqaureCrop);

            
            imshow("test1",sqaureCrop);
            
            f2d->detect( sqaureCrop, keypoints_2 );
            f2d->compute( sqaureCrop, keypoints_2, descriptors_2 );
            
            
            //-- Step 3: Matching descriptor vectors using BFMatcher :
            matcher.match( descriptors_1, descriptors_2, matches );
            
            // drawing the results
            namedWindow("matches", 1);
           
            if(matches.size() == 0)
            {
                sqaureCrop.release();
                continue;
            }
            else
                 cout<<"matches : " << matches.size()<<endl;
            
            Mat img_matches;
            drawMatches(db_original, keypoints_1, sqaureCrop, keypoints_2, matches, img_matches);
            imshow("matches", img_matches);
            
            cvWaitKey(100);
            sqaureCrop.release();
            
        }
 */
        cvWaitKey(20);
    }

    return 0;
}