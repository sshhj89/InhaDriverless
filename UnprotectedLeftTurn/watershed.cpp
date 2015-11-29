#include "MetaHeader.h"
#include "UV.h"

vector<int> theObject;
//bounding rectangle of the object, we will use the center of this as its position.
vector<Rect> objectBoundingRectangle;
//Rect objectBoundingRectangle2 = Rect(0,0,0,0); // 배열 처리

const static int SENSITIVITY_VALUE = 25;
const static int BLUR_SIZE = 15;

string intToString(int number){
    
    std::stringstream ss;
    ss << number;
    return ss.str();
}

class WatershedSegmenter {
private:
    Mat markers;
    
public:
    void setMarkers(const Mat& markerImage) {
        markerImage.convertTo(markers,CV_32S);

        // 정수형 영상 변환
    }
    
    Mat process(const Mat &image) {
        watershed(image,markers);
        // 워터쉐드 적용
        return markers;
    }
    
    Mat getSegmentation() { // 영상 형태인 결과를 반환
        Mat tmp;
        markers.convertTo(tmp,CV_8U);
        return tmp;
    }
    
     Mat getWatersheds() { // 영상 형태인 워터쉐드를 반환
        Mat tmp;
        markers.convertTo(tmp,CV_8U,255,255);
        return tmp;
    }
};

void searchForMovement(Mat thresholdImage, Mat &cameraFeed){
    
    bool objectDetected=false;
    Mat temp;
    thresholdImage.copyTo(temp);
    
    vector< vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours
    
    if(contours.size()>0)objectDetected=true;
    else objectDetected = false;
    
    if(objectDetected){
        //the largest contour is found at the end of the contours vector
        //we will simply assume that the biggest contour is the object we are looking for.
        vector<vector<Point>> largestContourVec;
        
        for(int i=0; i< contours.size(); i++)
            largestContourVec.push_back(contours.at(contours.size()-1-i));
        
        vector<vector<Point>>::const_iterator itLargestConture = largestContourVec.begin();
        
        theObject.clear();
        int xpos = 0;
        int ypos = 0;
        while(itLargestConture != largestContourVec.end())
        {
            objectBoundingRectangle.push_back(boundingRect((*itLargestConture)));
            xpos = boundingRect((*itLargestConture)).x+boundingRect((*itLargestConture)).width/2;
            ypos = boundingRect((*itLargestConture)).y+boundingRect((*itLargestConture)).height/2;
            
            theObject.push_back(xpos);
            theObject.push_back(ypos);
            ++itLargestConture;
        }
        
        
    }
    
    vector<int>::const_iterator itTheObject = theObject.begin();
    bool secondWhile = false;
    int x = 0;
    int y = 0;
    while(itTheObject != theObject.end())
    {
        if(secondWhile == false)
        {
            x= (*itTheObject);
            secondWhile = true;
            ++itTheObject;
            continue;
        }
        else
        {
            y = (*itTheObject);
            secondWhile = false;
        }
        
        circle(cameraFeed,Point(x,y),20,Scalar(0,255,0),2);
        line(cameraFeed,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
        line(cameraFeed,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
        line(cameraFeed,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
        line(cameraFeed,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
        putText(cameraFeed,"(" + intToString(x)+","+intToString(y)+")",Point(x,y),1,1,Scalar(255,0,0),2);
        
        ++itTheObject;
    }
}



int main()
{
    //ocl::setUseOpenCL(true);
    
    VideoCapture video("/Users/sonhojun/Downloads/data/all.mp4");
    
    if(!video.isOpened())
    {
        cout<<"video open error"<<endl;
        return 0 ;
    }
    Mat pri;
    Mat aft;
    Mat binary;
    Mat frame1,frame2;
    
    while (1)
    {
        
        video.read(frame1);
        video.read(frame2);
        
//        Mat cropFrame1, cropFrame2;
//        Mat oriConv = frame1(Rect(10,340,700,200));
//        
////        cropFrame1 = frame1(Rect(10,340,700,200));
////        cropFrame2 = frame2(Rect(10,340,700,200));
//        
//        Mat grayImage1,grayImage2;
//        Mat differenceImage;
//        Mat thresholdImage;
//        Vec<double,4> totalDiff = 0.0;
//        
//        cropFrame1 = frame1(Rect(10,340,700,200));
//        cvtColor(cropFrame1, grayImage1, COLOR_BGR2GRAY);
//        
//        cropFrame2 =frame2(Rect(10,340,700,200));
//        
//        cvtColor(cropFrame2, grayImage2, COLOR_BGR2GRAY);
//        absdiff(grayImage1,grayImage2,differenceImage);
//        
//        totalDiff = sum(differenceImage) / (690 * 140);
//        cout << "sum diff: " <<totalDiff<<endl;
//        
//        if(totalDiff[0] > 14.0)
//            continue;
//        
//        threshold(differenceImage, thresholdImage, SENSITIVITY_VALUE, 255, THRESH_BINARY);
//        
//        blur(thresholdImage, thresholdImage , Size(BLUR_SIZE,BLUR_SIZE));
//        
//        threshold(thresholdImage,thresholdImage,SENSITIVITY_VALUE,255,THRESH_BINARY);
//        
//        
//        //if tracking enabled, search for contours in our thresholded image
//
//        //searchForMovement(thresholdImage, oriConv);
//        
//        
//        //show our captured frame
//        imshow("moving",thresholdImage);
        
      
       // imshow("ori",temp);
        Mat cropRGB = frame2(Rect(10,340,700,200));
        Mat cropG;
        cvtColor(cropRGB, cropG, CV_RGB2GRAY);
        GaussianBlur(cropG, cropG, Size(5,5), 0,0);
        threshold(cropG,binary,0,255,THRESH_OTSU|THRESH_BINARY);
      //  adaptiveThreshold(cropG, binary, 255, <#int adaptiveMethod#>, THRESH_OTSU|THRESH_BINARY, 3, 0.);
       // bitwise_xor(thresholdImage,binary,binary);
        imshow("bin",binary);
      
        Mat fg;
        
        erode(binary, fg, Mat(), Point(-1, -1), 6);

        namedWindow("Foreground Image");
        imshow("Foreground Image", fg);

        Mat bg;
        dilate(binary,bg,Mat(),Point(-1,-1),6);
        threshold(bg,bg,1,128,THRESH_BINARY_INV);

        namedWindow("Background Image");
        imshow("Background Image",bg);

//        Mat dist;
//        distanceTransform(binary, dist, CV_DIST_L2, 5);
//        normalize(dist, dist, 0, 1., NORM_MINMAX);
//        imshow("dist",dist);
//        
//        threshold(dist, dist, .4, 1., CV_THRESH_BINARY);
//                // Dilate a bit the dist image
//        Mat kernel1 = Mat::ones(3, 3, CV_8UC1);
//        dilate(dist, dist, kernel1);
//        imshow("Peaks", dist);
        
        Mat markers(binary.size(),CV_8U,Scalar(0));
        markers= fg+bg;

        namedWindow("Markers");
        imshow("Markers",markers);
        
        WatershedSegmenter segmenter;
        
        segmenter.setMarkers(markers);
        segmenter.process(cropRGB);

        namedWindow("Segmentation");
        imshow("Segmentation",segmenter.getSegmentation());
        
       // segmenter.getSegmentation()
//        namedWindow("Watersheds"); // 워터쉐드 띄워 보기
//        imshow("Watersheds",segmenter.getWatersheds());
        
        
        
        
//111        Mat kernel = Mat::ones(3,3,CV_8U);
//        
//        Mat morpho;
//        morphologyEx(binary,morpho,MORPH_OPEN,kernel);
//        
//        imshow("open",morpho);
//        
//        Mat sure_bg;
//        dilate(morpho,sure_bg,kernel);
//        
//        imshow("dilate",sure_bg);
//        
//        Mat dist;
//        distanceTransform(binary, dist, CV_DIST_L2, 5);
//        // Normalize the distance image for range = {0.0, 1.0}
//        // so we can visualize and threshold it
//        normalize(dist, dist, 0, 1., NORM_MINMAX);
//        imshow("Distance Transform Image", dist);
//        // Threshold to obtain the peaks
//        // This will be the markers for the foreground objects
//        threshold(dist, dist, .4, 1., CV_THRESH_BINARY);
//        // Dilate a bit the dist image
//        Mat kernel1 = Mat::ones(3, 3, CV_8UC1);
//        dilate(dist, dist, kernel1);
//        imshow("Peaks", dist);
//        // Create the CV_8U version of the distance image
//        // It is needed for findContours()
//        Mat dist_8u;
//        dist.convertTo(dist_8u, CV_8U);
//        // Find total markers
//        vector<vector<Point> > contours;
//        findContours(dist_8u, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//        // Create the marker image for the watershed algorithm
//        Mat markers = Mat::zeros(dist.size(), CV_32SC1);
//        // Draw the foreground markers
//        for (size_t i = 0; i < contours.size(); i++)
//            drawContours(markers, contours, static_cast<int>(i), Scalar::all(static_cast<int>(i)+1), -1);
//        // Draw the background marker
//        circle(markers, Point(5,5), 3, CV_RGB(255,255,255), -1);
//        imshow("Markers", markers*10000);
//        // Perform the watershed algorithm
//        watershed(cropRGB, markers);
////        Mat mark = Mat::zeros(markers.size(), CV_8UC1);
////        markers.convertTo(mark, CV_8UC1);
////        bitwise_not(mark, mark);
////        imshow("Markers_v2", mark); // uncomment this if you want to see how the mark
//        // image looks like at that point
//        // Generate random colors
//        vector<Vec3b> colors;
//        for (size_t i = 0; i < contours.size(); i++)
//        {
//            int b = theRNG().uniform(0, 255);
//            int g = theRNG().uniform(0, 255);
//            int r = theRNG().uniform(0, 255);
//            colors.push_back(Vec3b((uchar)b, (uchar)g, (uchar)r));
//        }
//        // Create the result image
//        Mat dst = Mat::zeros(markers.size(), CV_8UC3);
//        // Fill labeled objects with random colors
//        for (int i = 0; i < markers.rows; i++)
//        {
//            for (int j = 0; j < markers.cols; j++)
//            {
//                int index = markers.at<int>(i,j);
//                if (index > 0 && index <= static_cast<int>(contours.size()))
//                    dst.at<Vec3b>(i,j) = colors[index-1];
//                else
//                    dst.at<Vec3b>(i,j) = Vec3b(0,0,0);
//            }
//        }
//        // Visualize the final image
//        imshow("Final Result", dst);
//        
//    
// 111       1

        cvWaitKey(10);
    }
    
    return 0;
}