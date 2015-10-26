//
//  Lane.cpp
//  UnprotectedLeftTurn
//
//  Created by SonHojun on 10/15/15.
//  Copyright (c) 2015 SonHojun. All rights reserved.
//

#include "Lane.h"

int bkTotLeft = 0;
int bkTotRight = 0;
int noiseLeft = 0;
int noiseRight = 0;

void Lane::findWhiteInImage(Mat& image, Mat& result)
{
    inRange(image,Scalar( WHITE_HUE_MIN, WHITE_SAT_MIN, WHITE_INT_MIN), Scalar(WHITE_HUE_MAX, WHITE_SAT_MAX, WHITE_INT_MAX), result);
}

void Lane::findYellowInImage(Mat& image, Mat& result)
{
    inRange(image,Scalar( YELLOW_HUE_MIN, YELLOW_SAT_MIN, YELLOW_INT_MIN), Scalar(YELLOW_HUE_MAX, YELLOW_SAT_MAX, YELLOW_INT_MAX),result);
}

vector<Vec4i> Lane::findLines(Mat& image, bool stop )
{
    lines.clear();
    positiveLines.clear();
    positiveInfo.clear();
    negativeInfo.clear();
    negativeLines.clear();
    HoughLinesP(image, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
    
    if(stop == true)
    {
        horizontalLines.clear();
        setLineLengthAndGap(70,20);  //line pixel and gap
        setMinVote(80);
        HoughLinesP(image, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
    }
    
    return lines;
    
}

void Lane::drawLine(Mat& image)
{
    vector<Vec4i>::const_iterator pos_it = positiveLines.begin();
    
    while(pos_it != positiveLines.end())
    {
        Point pt1((*pos_it)[0],(*pos_it)[1]);
        Point pt2((*pos_it)[2],(*pos_it)[3]);
        line(image,pt1,pt2,Scalar(255,0,0),2,CV_AA);
        ++pos_it;
    }
    
    vector<Vec4i>::const_iterator neg_it = negativeLines.begin();
    while(neg_it != negativeLines.end())
    {
        Point pt1((*neg_it)[0],(*neg_it)[1]);
        Point pt2((*neg_it)[2],(*neg_it)[3]);
        line(image,pt1,pt2,Scalar(0,255,0),2,CV_AA);
        ++neg_it;
    }
    
    vector<Vec4i>::const_iterator hor_it = horizontalLines.begin();
    while(hor_it != horizontalLines.end())
    {
        Point pt1((*hor_it)[0],(*hor_it)[1]);
        Point pt2((*hor_it)[2],(*hor_it)[3]);
        line(image,pt1,pt2,Scalar(255,255,255),3,CV_AA);
        //cout<<" S   T    O   P" << endl;
        
        ++hor_it;
    }
    
}

void Lane::filterLine(Mat& image,double myX,double myY)
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
        
        
        if( (lineSlope < 100. && lineSlope > 0.2 ) && pt1X < 0.)  //lines at left
        {
            interceptY = calInterceptY(pt1Y, lineSlope, pt1X);
            interceptX = calInterceptX(interceptY, lineSlope);
            
            /* filter by rho */
            perSlope = perpendicular(lineSlope);
            
            interSectX = interceptY / (lineSlope - perSlope);
            interSectY = interSectX * perSlope;
            
            rho = sqrt(pow(interSectX,2) + pow(interSectY,2));
            
            
            if(rho > bandLeftMax|| rho < bandLeftMin)
            {
                ++it;
                continue;
            }
            
            positiveInfo.push_back(LineInfo(rho, lineSlope, \
                                            interceptX, interceptY, \
                                            Point((*it)[0],(*it)[1]), Point((*it)[2],(*it)[3])));
            positiveLines.push_back((*it));
        }
        else if(lineSlope < - 0.4 && pt1X > 0.)  // lines at right
        {
            
            interceptY = calInterceptY(pt1Y, lineSlope, pt1X);
            interceptX = calInterceptX(interceptY, lineSlope);
            /*  filter by rho */
            perSlope = perpendicular(lineSlope);
            perIntercept = calInterceptY(0, perSlope, 0);
            
            interSectX = interceptY / (lineSlope - perSlope);
            interSectY = interSectX * perSlope;
            
            rho = sqrt(pow(interSectX,2) + pow(interSectY,2));
            
            
            if(rho > bandRightMax || rho < bandRightMin)
            {
                ++it;
                continue;
            }
            negativeInfo.push_back(LineInfo(rho, lineSlope, \
                                            interceptX, interceptY, Point((*it)[0],(*it)[1]), Point((*it)[2],(*it)[3])));
            negativeLines.push_back((*it));
        }
        else if(lineSlope > - 0.1 && lineSlope < 0.1)  //hori
        {
            horizontalLines.push_back((*it));
        }
        
        ++it;
    }
}


void Lane::filterStopLine()
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
        
        it_posi = positiveLines.begin();
        while(it_posi != positiveLines.end())
        {
            double posix1 = (*it_posi)[0];
            double posix2 = (*it_posi)[2];
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



int Lane::kmeansPositive()
{
    
    if(positiveInfo.size() == 0 || positiveInfo.size() == 1)
    {
        cout<<"return 1"<<endl;
        return 1; // just one line detected or not
    
    }
    
    double rhoMax = 0.;
    double rhoMin = RHOMAX;
    bool findFlag = false;
    vector<double> datas;
    vector<double> class1;
    vector<double> class2;
    
    vector<LineInfo>::const_iterator it = positiveInfo.begin();
    
    /*** class init ****/
    while(it != positiveInfo.end())
    {
        if(rhoMax < (*it).getRho())
        {
            class1.clear();
            rhoMax = (*it).getRho();
            class1.push_back((*it).getSlope());
            class1.push_back((*it).getIntX());
            class1.push_back((*it).getIntY());
            class1.push_back((*it).getRho());
            
        }
        
        if(rhoMin >= (*it).getRho())
        {
            class2.clear();
            rhoMin = (*it).getRho();
            class2.push_back((*it).getSlope());
            class2.push_back((*it).getIntX());
            class2.push_back((*it).getIntY());
            class2.push_back((*it).getRho());
        }
        
        ++it;
    }
    
    if(class2.size() == 0 || class1.size() == 0)
    {
        cout<<"hererererere"<<endl;
        return 2;
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
        double sum = 0.;
        while(it != positiveInfo.end())
        {
            total4class1 += ((*it).getSlope() - class1[0]) * 100 + ((*it).getIntX() - class1[1]) + \
            ((*it).getIntY() - class1[2]) +((*it).getRho() - class1[3]);
            
            total4class2 += ((*it).getSlope() - class2[0]) * 100 + ((*it).getIntX() - class2[1]) + \
            ((*it).getIntY() - class2[2]) +((*it).getRho() - class2[3]);
            
            if(abs(total4class1) <= abs(total4class2)) // assign class1;
            {
                sum4Class1[0] += (*it).getSlope();
                sum4Class1[1] += (*it).getIntX();
                sum4Class1[2] += (*it).getIntY();
                sum4Class1[3] += (*it).getRho();
                count4Class1++;
                
            }
            else
            {
                sum4Class2[0] += (*it).getSlope();
                sum4Class2[1] += (*it).getIntX();
                sum4Class2[2] += (*it).getIntY();
                sum4Class2[3] += (*it).getRho();
                count4Class2++;
            }
            
            /**** init ****/
            total4class1 = total4class2 = 0.;
            
            ++it;
        }
        
        
        /*** cetner  구하기 ***/
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
    
    if(findFlag == true)  //find & classify
    {
        
    }
    
    double sumClass1 = 0.;
    double sumClass2 = 0.;
    
    sumClass1 = class1[0] * 100. + class1[1] + class1[2] + class1[3];
    
    sumClass2 = class2[0] * 100. + class2[1] + class2[2] + class2[3];
    cout<<"diffff: " <<abs(sumClass1 - sumClass2)<<endl;
   
    if(abs(sumClass1 - sumClass2) < 60.)
    {
        cout<<" no Cluster " <<abs(sumClass1 - sumClass2)<<endl;
        return 2; //fail to cluster
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
        sumData = (*it).getRho() + (*it).getSlope() * 100.0 + (*it).getIntX() + (*it).getIntY();
        if( abs(sumClass1 - sumData) < abs(sumClass2 - sumData))
        {
            
            temp[0] = (*it).getPoint1X();
            temp[1] = (*it).getPoint1Y();
            temp[2] = (*it).getPoint2X();
            temp[3] = (*it).getPoint2Y();
            
            //longest line
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
            temp[0] = (*it).getPoint1X();
            temp[1] = (*it).getPoint1Y();
            temp[2] = (*it).getPoint2X();
            temp[3] = (*it).getPoint2Y();
            
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
    
    positiveLines.clear();
    positiveLines.push_back(max);
    positiveLines.push_back(min);
    
    positiveInfo.clear();
    positiveInfo.push_back(tempMax.front());
    positiveInfo.push_back(tempMin.front());
    
    return 3; //success cluster
}

double Lane::findMedianRho(int direction)
{
    if(direction == 1) //pos
    {
        if(positiveLines.size() == 0)
        {
            return -0.1;
        }
        else if (positiveLines.size() == 1)
        {
            return positiveInfo.at(0).getRho();
        }
        else if (positiveInfo.size() == 2)
        {
            return (positiveInfo.at(0).getRho() + positiveInfo.at(1).getRho()) / 2.;
        }
        else
        {
            vector<LineInfo>::const_iterator it = positiveInfo.begin();
            double rhoVector[positiveInfo.size()];
            int i=0;
            while(it != positiveInfo.end())
            {
                rhoVector[i++] = (*it).getRho();
                ++it;
            }
            
            sort(rhoVector,rhoVector+positiveInfo.size());
            if(positiveInfo.size() % 2 == 0) //짝수
            {
                return (rhoVector[positiveInfo.size()/2] + rhoVector[positiveInfo.size()/2+1]) / 2.;
            }
            else
            {
                return rhoVector[positiveInfo.size()/2+1];
            }
            
        }
    }
    else if(direction == 2) // neg
    {
        if(negativeLines.size() == 0)
        {
            return -0.1;
        }
        else if (negativeLines.size() == 1)
        {
            return negativeInfo.at(0).getRho();
        }
        else if (negativeInfo.size() == 2)
        {
            return (negativeInfo.at(0).getRho() + negativeInfo.at(1).getRho()) / 2.;
        }
        else
        {
            vector<LineInfo>::const_iterator it = negativeInfo.begin();
            double rhoVector[negativeInfo.size()];
            int i=0;
            while(it != negativeInfo.end())
            {
                rhoVector[i++] = (*it).getRho();
                ++it;
            }
            
            sort(rhoVector,rhoVector+negativeInfo.size());
            if(negativeInfo.size() % 2 == 0) //짝수
            {
                return (rhoVector[negativeInfo.size()/2] + rhoVector[negativeInfo.size()/2+1]) / 2.;
            }
            else
            {
                return rhoVector[negativeInfo.size()/2+1];
            }
        }
    }
    
    return -0.1;
    
}

int Lane::laneChecker()
{
    
    list<double>::const_iterator itpos = this->getBoundRhoPos().begin();
    
    while(itpos != this->getBoundRhoPos().end())
    {
        cout<<"pos: " << (*itpos)<<" ";
        (itpos)++;
    }
    cout<<endl;
    
    list<double>::const_iterator itneg = this->getBoundRhoNeg().begin();
    while(itneg != this->getBoundRhoNeg().end())
    {
        cout<<"neg: " << (*itneg)<< " ";
        ++itneg;
    }
    
    cout<<endl;
    
    if(getCurrentRhoPos() - this->getBoundRhoPos().front() > DIFF && getBkRhoPos() != -0.1)
    {
        if(this->getBoundRhoPos().front() == 0.)
            return 0;
        cout<<"1. currentRho: " << getCurrentRhoPos() <<", BoundRhopos: " << getBoundRhoPos().front() << endl;
        boundaryCountLeft1++;
        this->getBoundRhoPos().pop_front();
        
    }else if( getCurrentRhoNeg() - this->getBoundRhoNeg().front() > DIFF && getBkRhoNeg() != -0.1)
    {
        if(this->getBoundRhoNeg().front() == 0.)
            return 0;
        cout<<"2. currentRho: " << getCurrentRhoNeg() <<", BoundRhoNeg: " << getBoundRhoNeg().front() << endl;
        boundaryCountRight1++;
        this->getBoundRhoNeg().pop_front();
    }
    
    
    if(this->getBoundRhoNeg().front() - getCurrentRhoNeg() > DIFF && getBkRhoNeg() != -0.1)
    {
        if(this->getBoundRhoNeg().front() == 0.)
            return 0;
        
        cout<<"3. currentRho: " << getCurrentRhoNeg() <<", BoundRhoNeg: " << getBoundRhoNeg().front() << endl;
        boundaryCountLeft2++;
        this->getBoundRhoNeg().pop_front();
    } else if(this->getBoundRhoPos().front() - getCurrentRhoPos() > DIFF && getBkRhoPos() != -0.1)
    {
        if(this->getBoundRhoPos().front() == 0.)
            return 0;
        
        cout<<"4.currentRho: " << getCurrentRhoPos() <<", BoundRhopos: " << getBoundRhoPos().front() << endl;
        boundaryCountRight2++;
        this->getBoundRhoPos().pop_front();
        
    }
    
    int totLeft = boundaryCountLeft1 + boundaryCountLeft2;
    int totRight = boundaryCountRight1 + boundaryCountRight2;
    
    
    if(bkTotLeft == totLeft)
    {
        noiseLeft++;
    }
    else{
        noiseLeft = 0;
        bkTotLeft = boundaryCountLeft1 + boundaryCountLeft2;
    }
    
    if(bkTotRight == totRight)
    {
        noiseRight++;
    }
    else
    {
        noiseRight = 0;
        bkTotRight = boundaryCountRight1 + boundaryCountRight2;

    }
    
    if(noiseLeft == BOUNDINIT*2)
    {
        noiseLeft = 0;
        boundaryCountLeft1 = 0;
        boundaryCountLeft2 = 0;
    }
    
    if(noiseRight == BOUNDINIT*2)
    {
        noiseRight = 0;
        boundaryCountRight1 = 0;
        boundaryCountRight2 = 0;
    }
    
    cout<<"totLeft : " << totLeft <<" , "<<"totRight: " <<totRight<<endl;
    
    if(totLeft == BOUNDINIT)
    {
        this->getBoundRhoNeg().clear();
        this->getBoundRhoPos().clear();
        
        boundaryCountLeft1 = 0;
        boundaryCountLeft2 = 0;
        boundaryCountRight1 = 0;
        boundaryCountRight2 = 0;
        cout<<"change lane to left" << endl;
        noiseLeft = 0;
        noiseRight = 0;
        return -1;
    }
    else if (totRight == BOUNDINIT)
    {
        boundaryCountLeft1 = 0;
        boundaryCountLeft2 = 0;
        boundaryCountRight1 = 0;
        boundaryCountRight2 = 0;
        this->getBoundRhoPos().clear();
        this->getBoundRhoNeg().clear();
        
        noiseLeft = 0;
        noiseRight = 0;
        
        
        cout << "change lane to right" << endl;
        return 1;
    }
   
    BOUNDCOUNT++;
    
    return 0; //keep current state
}



