//
// Created by intelnuc on 23-6-6.
//

#include "../include/record.h"
#include "sys/time.h"
#include "time.h"

using namespace std;
using namespace cv;

void record :: init_record(int model)
{
    char time[40];
    struct timeval tv;
    struct tm* ptm;
    char time_string[40];
    long milliseconds;

    gettimeofday(&tv,NULL);
    ptm = localtime(&(tv.tv_sec));

    strftime(time_string,sizeof(time_string),"%Y-%m-%d%H:%M:%S",ptm);
    milliseconds = tv.tv_usec/1000;
    snprintf(time,sizeof(time),"%s.%03ld",time_string,milliseconds);

    string name(time);

    if(model==0)
    path = "../record/Original"+name+".avi";
    else
        path =  "../record/After"+name+".avi";
    outputVideo.open(path,VideoWriter::fourcc('X','V','I','D'),fps,S,true);
}

void record :: start_record(Mat frame)
{
    outputVideo<<frame;
}

