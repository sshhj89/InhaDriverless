

#ifndef __CTH__
#define __CTH__
#include "kalman.h"
#include "HungarianAlg.h"
#include "MetaHeader.h"
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

class CTrack
{
public:
    vector<Point2d> trace;
    static size_t NextTrackID;
    size_t track_id;
    size_t skipped_frames;
    Point2f prediction;
    TKalmanFilter* KF;
    CTrack(Point2f p, float dt, float Accel_noise_mag);
    ~CTrack();
};


class CTracker
{
public:
    
    float dt;
    
    float Accel_noise_mag;
    
    double dist_thres;
    int maximum_allowed_skipped_frames;
    int max_trace_length;
    
    vector<CTrack*> tracks;
    void Update(vector<Point2f>& detections);
    CTracker(float _dt, float _Accel_noise_mag, double _dist_thres=60, int _maximum_allowed_skipped_frames=2,int _max_trace_length=15);
    ~CTracker(void);
};

#endif