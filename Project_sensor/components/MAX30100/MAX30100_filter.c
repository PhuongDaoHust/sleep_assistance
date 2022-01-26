#include "MAX30100_filter.h"
// http://sam-koblenski.blogspot.de/2015/11/everyday-dsp-for-programmers-dc-and.html
float DC_Remover(float x){
	float olddcw = dcw;
	dcw = (float)x + DC_REMOVER_ALPHA*(dcw);
	return dcw - olddcw;
}
float DC_R(float x){
	return x;
}
// http://www.schwietering.com/jayduino/filtuino/
// Low pass butterworth filter order=1 alpha1=0.1
// Fs=100Hz, Fc=6Hz
float FilterBuLp1(float x){
	v_buff[0] = v_buff[1];
	v_buff[1] = (2.452372752527856026e-1 * x) + (0.50952544949442879485 * v_buff[0]);
	return (v_buff[0] + v_buff[1]);
}	