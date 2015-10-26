//  UV.cpp
//  UnprotectedLeftTurn
//
//  Created by SonHojun on 10/18/15.
//  Copyright (c) 2015 SonHojun. All rights reserved.
//

#include "UV.h"

void UVCar::modifyOnLane(int l)
{
    if(possibleLine == true)
    {
        if(l == -1) return;
        else if( l == 0) return;
        else if( l == 1)
        {
            onLane = l;
            possibleLine = false;
        }
    }
    else
    {
        onLane += l;
    }
}




