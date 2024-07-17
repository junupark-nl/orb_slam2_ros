#ifndef ORBPARAMETERS_H
#define ORBPARAMETERS_H

namespace ORB_SLAM2
{
    struct ORBParameters{
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
        float baseline;
        int fps;
        bool isRGB;

        // depth-involved
        float depthMapFactor;
        float thDepth;
    };
}

#endif // ORBPARAMETERS_H