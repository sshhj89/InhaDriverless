#include "LineFinder.h"

#define RHOMAX 200.0
#define RHOMIN 130.0

vector<Vec4i> LineFinder::findLines(UMat& bin, int pnha)
{
    lines.clear();
    if(pnha == 0)
    {
        positiveLines.clear();
        positiveInfo.clear();
    }
    else if(pnha == 1)
    {
        negativeLines.clear();
        negativeInfo.clear();
    }
    else if (pnha == 2)
        horizontalLines.clear();
    else
    {
        positiveLines.clear();
        positiveInfo.clear();
        
        negativeLines.clear();
        negativeInfo.clear();
        
        horizontalLines.clear();
    }
    
    HoughLinesP(bin, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
    return lines;
    
}


void LineFinder::filterLine(UMat& image, int myX , int myY, int pnha )
{
    vector<Vec4i>::const_iterator it = lines.begin();
    
    double pt1X = 0.;
    double pt1Y = 0.;
    double pt2X = 0.;
    double pt2Y = 0.;
    
    double rho = 0. ;
    double lineSlope = 0.;
    
    double interceptY = 0.;
    double interceptX = 0.;
    
    double perSlope = 0.;
    double perIntercept = 0.;
    double interSectX = 0.;
    double interSectY = 0.;
    
    while(it != lines.end())
    {
        pt1X =  (*it)[0] - myX;
        pt1Y =  myY - (*it)[1];
        pt2X =  (*it)[2] - myX;
        pt2Y =  myY - (*it)[3];
        
        lineSlope = calSlope(pt2Y, pt1Y, pt2X, pt1X);
        
        if( (lineSlope < 1. && lineSlope > 0.5 ) && pt1X < 0. && (pnha == 0 || pnha == 3))  //lines at left
        {
            interceptY = calInterceptY(pt1Y, lineSlope, pt1X);
            interceptX = calInterceptX(interceptY, lineSlope);
            
            /* rho 로 필터링 하기 */
            perSlope = perpendicular(lineSlope);
            
            interSectX = interceptY / (lineSlope - perSlope);
            interSectY = interSectX * perSlope;
            
            rho = sqrt(pow(interSectX,2) + pow(interSectY,2));
            
            
            if(rho > RHOMAX || rho < RHOMIN)
            {
                ++it;
                continue;
            }
            //cout<<"rho: " <<rho<<endl;
            positiveInfo.push_back(LineInfo(rho, lineSlope, \
                                            interceptX, interceptY, Point((*it)[0],(*it)[1]), Point((*it)[2],(*it)[3])));
            //positiveLines.push_back((*it));
            
        }
        else if(lineSlope < - 0.78 && pt1X > 0. && (pnha == 1 || pnha == 3))  // lines at right
        {
            
            interceptY = calInterceptY(pt1Y, lineSlope, pt1X);
            interceptX = calInterceptX(interceptY, lineSlope);
            /* rho 로 필터링 하기 */
            perSlope = perpendicular(lineSlope);
            //cout <<"perSlope: " << perSlope << endl;
            perIntercept = calInterceptY(0, perSlope, 0);
            //cout<<"perInterCept: " << perIntercept << endl;
            
            interSectX = interceptY / (lineSlope - perSlope);
            interSectY = interSectX * perSlope;
            
            rho = sqrt(pow(interSectX,2) + pow(interSectY,2)); 
            if(rho > RHOMAX || rho < RHOMIN)
            {
                ++it;
                continue;
            }
            //cout<<"rho : " << rho<<endl;
            negativeInfo.push_back(LineInfo(rho, lineSlope, \
                                            interceptX, interceptY, Point((*it)[0],(*it)[1]), Point((*it)[2],(*it)[3])));
            //cout<<"size: " <<negativeInfo.size()<<endl;
        }
        else if(lineSlope > - 0.1 && lineSlope < 0.1 && (pnha == 2 || pnha == 3))  //hori
        {
            horizontalLines.push_back((*it));
        }
        ++it;
    }
}

bool LineFinder::kmeansPositive() //return rho
{

    if(positiveInfo.size() == 0)
        return false;
    
    double rhoMax = 0.;
    double rhoMin = 200.;
    bool findFlag = false;
    vector<double> datas;
    vector<double> class1;
    vector<double> class2;
    
    vector<LineInfo>::const_iterator it = positiveInfo.begin();
    
    /*** class init ****/
    while(it != positiveInfo.end())
    {
        if(rhoMax < (*it).rho)
        {
            class1.clear();
            rhoMax = (*it).rho;
            class1.push_back((*it).slope);
            class1.push_back((*it).interceptX);
            class1.push_back((*it).interceptY);
            class1.push_back((*it).rho);
            
        }
        
        if(rhoMin > (*it).rho)
        {
            class2.clear();
            rhoMin = (*it).rho;
            class2.push_back((*it).slope);
            class2.push_back((*it).interceptX);
            class2.push_back((*it).interceptY);
            class2.push_back((*it).rho);
        }
        
        ++it;
    }
    
    
    
    double total4class1 = 0.;
    double total4class2 = 0.;
    
    double sum4Class1[FEATURES] = {0.,};
    int count4Class1 = 0;
    double center4Class1[FEATURES] = {0.,};
    
    double sum4Class2[FEATURES] = {0.,};
    int count4Class2 = 0;
    double center4Class2[FEATURES] = {0.,};
    
    for(int testNum = 0; testNum<TESTNUM ; testNum++)
    {
        
        it = positiveInfo.begin();
        
        while(it != positiveInfo.end())
        {
            total4class1 += ((*it).slope - class1[0]) * 1000 + ((*it).interceptX - class1[1]) + \
            ((*it).interceptY - class1[2]) +((*it).rho - class1[3]);
            
            total4class2 += ((*it).slope - class2[0]) * 1000 + ((*it).interceptX - class2[1]) + \
            ((*it).interceptY - class2[2]) +((*it).rho - class2[3]);
            
            if(abs(total4class1) <= abs(total4class2)) // assign class1;
            {
                sum4Class1[0] += (*it).slope;
                sum4Class1[1] += (*it).interceptX;
                sum4Class1[2] += (*it).interceptY;
                sum4Class1[3] += (*it).rho;
                count4Class1++;
                
            }
            else
            {
                sum4Class2[0] += (*it).slope;
                sum4Class2[1] += (*it).interceptX;
                sum4Class2[2] += (*it).interceptY;
                sum4Class2[3] += (*it).rho;
                count4Class2++;
            }
            
            /**** 초기화 ****/
            total4class1 = total4class2 = 0.;
            
            ++it;
        }
        
        
        /*** 중심 좌표 구하기 ***/
        for(int i=0;i<FEATURES;i++)
        {
            center4Class1[i] = sum4Class1[i] / count4Class1;
            center4Class2[i] = sum4Class2[i] / count4Class2;
        }
        
        /*** check class with center4Class *****/
        double checkSumClass1 = 0.;
        double checkSumClass2 = 0.;
        
        for(int i=0;i<FEATURES;i++)
        {
            checkSumClass1 += class1[i] - center4Class1[i];
            checkSumClass2 += class2[i] - center4Class2[i];
        }
        
        if(checkSumClass1 != 0.)
        {
            for(int i=0; i<FEATURES; i++)
            {
                class1[i] = center4Class1[i];
            }
        }
        else
        {
            findFlag = true;
            break;
        }
        
        if(checkSumClass2 != 0.)
        {
            for(int i=0; i<FEATURES; i++)
            {
                class2[i] = center4Class2[i];
            }
        }
        else
        {
            findFlag = true;
            break;
            
        }
        
        for(int i = 0; i < FEATURES; i++)
        {
            sum4Class1[i] = 0.0;
            sum4Class2[i] = 0.0;
        }
        
        count4Class1 = 0;
        count4Class2 = 0;
        
    }
    
    if(findFlag == true)  //찾았고 분리 완료
    {
        
    }
    
    double sumClass1 = 0.;
    double sumClass2 = 0.;
    
    sumClass1 = class1[0] * 100. + class1[1] + class1[2] + class1[3];
    sumClass2 = class2[0] * 100. + class2[1] + class2[2] + class2[3];
    
    if(abs(sumClass1 - sumClass2) < 5. )
    {
        //cout<<" no Cluster " <<endl;   //2,3차 라인인 것을 알 수 있다.
        //cout<<"size: " <<positiveInfo.size()<<endl;
        return false;
    }
    
    it = positiveInfo.begin();
    double sumData = 0.;
    double euDistance1 = 0.;
    double lineDistance1 = 0.;
    double euDistance2 = 0.;
    double lineDistance2 = 0.;
    
    Vec4i temp;
    Vec4i max;
    Vec4i min;
    
    vector<LineInfo> tempMax;
    vector<LineInfo> tempMin;
    
    while(it != positiveInfo.end())
    {
        sumData = (*it).rho + (*it).slope * 100.0 + (*it).interceptX + (*it).interceptY;
        if( abs(sumClass1 - sumData) < abs(sumClass2 - sumData))
        {
            
            temp[0] = (*it).pt1.x;
            temp[1] = (*it).pt1.y;
            temp[2] = (*it).pt2.x;
            temp[3] = (*it).pt2.y;
            
            //가장 긴 라인
            lineDistance1 = sqrt(pow(temp[0]-temp[2],2) + pow(temp[1]-temp[3],2));
            if(lineDistance1 > euDistance1)
            {
                euDistance1 = lineDistance1;
                max = temp;
                tempMax.clear();
                tempMax.push_back((*it));
            }
        }
        else
        {
            temp[0] = (*it).pt1.x;
            temp[1] = (*it).pt1.y;
            temp[2] = (*it).pt2.x;
            temp[3] = (*it).pt2.y;
            
            
            lineDistance2 = sqrt(pow(temp[0]-temp[2],2) + pow(temp[1]-temp[3],2));
            if(lineDistance2 > euDistance2)
            {
                euDistance2 = lineDistance2;
                min = temp;
                tempMin.clear();
                tempMin.push_back((*it));
                
            }
            
        }
        
        ++it;
    }
    
    positiveLines.push_back(max);
    positiveLines.push_back(min);
    
    positiveInfo.clear();
    positiveInfo.push_back(tempMax.front());
    positiveInfo.push_back(tempMin.front());
    
    return true;
}

bool LineFinder::kmeansNegative()
{
    
    if(negativeInfo.size() == 0)
        return false;
    
    double rhoMax = 0.;
    double rhoMin = 200.;
    bool findFlag = false;
    vector<double> datas;
    vector<double> class1;
    vector<double> class2;
    vector<LineInfo> tempNegative;
    
    vector<LineInfo>::const_iterator it = negativeInfo.begin();
    
    /*** class init ****/
    while(it != negativeInfo.end())
    {
        if(rhoMax < (*it).rho)
        {
            class1.clear();
            rhoMax = (*it).rho;
            class1.push_back((*it).slope);
            class1.push_back((*it).interceptX);
            class1.push_back((*it).interceptY);
            class1.push_back((*it).rho);
            
        }
        
        if(rhoMin > (*it).rho)
        {
            class2.clear();
            rhoMin = (*it).rho;
            class2.push_back((*it).slope);
            class2.push_back((*it).interceptX);
            class2.push_back((*it).interceptY);
            class2.push_back((*it).rho);
        }
        
        ++it;
    }
    
    
    
    double total4class1 = 0.;
    double total4class2 = 0.;
    
    double sum4Class1[FEATURES] = {0.,};
    int count4Class1 = 0;
    double center4Class1[FEATURES] = {0.,};
    
    double sum4Class2[FEATURES] = {0.,};
    int count4Class2 = 0;
    double center4Class2[FEATURES] = {0.,};
    
    
    
    for(int testNum = 0; testNum<TESTNUM ; testNum++)
    {
        
        it = negativeInfo.begin();
        
        while(it != negativeInfo.end())
        {
            total4class1 += ((*it).slope - class1[0]) * 1000 + ((*it).interceptX - class1[1]) + \
            ((*it).interceptY - class1[2]) +((*it).rho - class1[3]);
            
            total4class2 += ((*it).slope - class2[0]) * 1000 + ((*it).interceptX - class2[1]) + \
            ((*it).interceptY - class2[2]) +((*it).rho - class2[3]);
            
            if(abs(total4class1) <= abs(total4class2)) // assign class1;
            {
                sum4Class1[0] += (*it).slope;
                sum4Class1[1] += (*it).interceptX;
                sum4Class1[2] += (*it).interceptY;
                sum4Class1[3] += (*it).rho;
                count4Class1++;
                
            }
            else
            {
                sum4Class2[0] += (*it).slope;
                sum4Class2[1] += (*it).interceptX;
                sum4Class2[2] += (*it).interceptY;
                sum4Class2[3] += (*it).rho;
                count4Class2++;
                
                
            }
            
            /**** 초기화 ****/
            total4class1 = total4class2 = 0.;
            
            ++it;
        }
        
        
        /*** 중심 좌표 구하기 ***/
        for(int i=0;i<FEATURES;i++)
        {
            center4Class1[i] = sum4Class1[i] / count4Class1;
            center4Class2[i] = sum4Class2[i] / count4Class2;
        }
        
        /*** check class with center4Class *****/
        double checkSumClass1 = 0.;
        double checkSumClass2 = 0.;
        
        for(int i=0;i<FEATURES;i++)
        {
            checkSumClass1 += class1[i] - center4Class1[i];
            checkSumClass2 += class2[i] - center4Class2[i];
        }
        
        if(checkSumClass1 != 0.)
        {
            for(int i=0; i<FEATURES; i++)
            {
                class1[i] = center4Class1[i];
            }
        }
        else
        {
            findFlag = true;
            break;
        }
        
        if(checkSumClass2 != 0.)
        {
            for(int i=0; i<FEATURES; i++)
            {
                class2[i] = center4Class2[i];
            }
        }
        else
        {
            findFlag = true;
            break;
            
        }
        
        for(int i = 0; i < FEATURES; i++)
        {
            sum4Class1[i] = 0.0;
            sum4Class2[i] = 0.0;
        }
        
        count4Class1 = 0;
        count4Class2 = 0;
        
    }
    
    if(findFlag == true)  //찾았고 분리 완료
    {
        
    }
    
    double sumClass1 = 0.;
    double sumClass2 = 0.;
    
    sumClass1 = class1[0] * 100. + class1[1] + class1[2] + class1[3];
    sumClass2 = class2[0] * 100. + class2[1] + class2[2] + class2[3];
    
    if(abs(sumClass1 - sumClass2) < 5. )
    {
       //cout<<" no Cluster " <<endl;   //2,3차 라인인 것을 알 수 있다.
        return false;
    }
    
    it = negativeInfo.begin();
    double sumData = 0.;
    double euDistance = 0.;
    double lineDistance = 0.;
    Vec4i temp;
    Vec4i max;
    Vec4i min;
    
    while(it != negativeInfo.end())
    {
        sumData = (*it).rho + (*it).slope * 100.0 + (*it).interceptX + (*it).interceptY;
        if( abs(sumClass1 - sumData) < abs(sumClass2 - sumData))
        {
            
            temp[0] = (*it).pt1.x;
            temp[1] = (*it).pt1.y;
            temp[2] = (*it).pt2.x;
            temp[3] = (*it).pt2.y;
            
            //가장 긴 라인
            lineDistance = sqrt(pow(temp[0]-temp[2],2) + pow(temp[1]-temp[3],2));
            if(lineDistance > euDistance)
            {
                euDistance = lineDistance;
                max = temp;
                tempNegative.clear();
                tempNegative.push_back((*it));
            }
        }
        
        ++it;
    }
    
    negativeLines.push_back(max);
    negativeInfo.clear();
    
    negativeInfo.push_back(tempNegative.front());
    return true;
    
}

void LineFinder::filterStopLine()
{
    vector<Vec4i> filteredHori;
    filteredHori.clear();
    
    vector<Vec4i>::const_iterator it_posi;
    vector<Vec4i>::const_iterator it_hori;

    
    it_hori = horizontalLines.begin();
    while(it_hori != horizontalLines.end())
    {
        double horix1 = (*it_hori)[0];
        double horix2 = (*it_hori)[2];
        
        //cout<<"hori : "<<horix1 <<" , " <<horix2<<endl;
        it_posi = positiveLines.begin();
        while(it_posi != positiveLines.end())
        {
            double posix1 = (*it_posi)[0];
            double posix2 = (*it_posi)[2];
            //cout<< "posi: " << posix1<< " , " << posix2<< endl;
            if( horix1 < posix2)
            {
                 ++it_posi;
                continue;
            }
            else
                filteredHori.push_back((*it_hori));
            
            ++it_posi;
        }
        
        ++it_hori;
    }
    
    horizontalLines.clear();
    
    vector<Vec4i>::const_iterator it_filteredhori = filteredHori.begin();
    
    while(it_filteredhori != filteredHori.end())
    {
        horizontalLines.push_back((*it_filteredhori));
        ++it_filteredhori;
    }

}
void LineFinder::drawLine(UMat& image)
{
    vector<Vec4i>::const_iterator it = positiveLines.begin();
    
    while(it != positiveLines.end())
    {
        Point pt1((*it)[0],(*it)[1]);
        Point pt2((*it)[2],(*it)[3]);
        line(image,pt1,pt2,Scalar(255),2,CV_AA);
        ++it;
    }
    
    it = negativeLines.begin();
    while(it != negativeLines.end())
    {
        Point pt1((*it)[0],(*it)[1]);
        Point pt2((*it)[2],(*it)[3]);
        line(image,pt1,pt2,Scalar(0,255,0),2,CV_AA);
        ++it;
    }
    
    it = horizontalLines.begin();
    while(it != horizontalLines.end())
    {
        Point pt1((*it)[0],(*it)[1]);
        Point pt2((*it)[2],(*it)[3]);
        line(image,pt1,pt2,Scalar(0,0,255),2,CV_AA);
        //cout<<" S   T    O   P" << endl;

        ++it;
    }
}