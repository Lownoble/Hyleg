/*
 * mathTools.c
 *
 */

#include "mathTools.h"

float saturation(float a, float limit1, float limit2){
    float lowLim, highLim;
	if(limit1 > limit2){
        lowLim = limit2;
        highLim= limit1;
    }else{
        lowLim = limit1;
        highLim= limit2;
    }

    if(a < lowLim){
        return lowLim;
    }
    else if(a > highLim){
        return highLim;
    }
    else{
        return a;
    }
}
