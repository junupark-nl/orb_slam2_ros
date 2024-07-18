#ifndef TRACKINGPARAMETERS_H
#define TRACKINGPARAMETERS_H

namespace ORB_SLAM2
{
    struct TrackingParameters{
        // ORB Extractor parameters
        int nFeatures;
        float scaleFactor;
        int nLevels;
        int iniThFAST;
        int minThFAST;

        // camera parameters 
        float fx;
        float fy;
        float cx;
        float cy;
        float k1;
        float k2;
        float p1;
        float p2;
        float k3;
        int fps;
        bool isRGB;

        // depth-involved
        float baseline;
        float depthMapFactor;
        float thDepth;
    };
}

#endif // TRACKINGPARAMETERS_H