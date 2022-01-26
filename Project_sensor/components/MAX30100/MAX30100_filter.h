#ifndef MAX30100_FILTERS_H
#define MAX30100_FILTERS_H

#define DC_REMOVER_ALPHA                    0.95
float v_buff[2];
float dcw;

// http://sam-koblenski.blogspot.de/2015/11/everyday-dsp-for-programmers-dc-and.html
float DC_Remover(float x);
float DC_R(float x);
// http://www.schwietering.com/jayduino/filtuino/
// Low pass butterworth filter order=1 alpha1=0.1
// Fs=100Hz, Fc=6Hz
float FilterBuLp1(float x);
#endif
