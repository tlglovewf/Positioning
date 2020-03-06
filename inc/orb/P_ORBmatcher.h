


#ifndef _ORBMATCHER_H_
#define _ORBMATCHER_H_

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"P_ORBMapPoint.h"
#include"P_ORBKeyFrame.h"
#include"P_ORBFrame.h"


namespace Position
{

class ORBmatcher
{    
public:

    ORBmatcher(float nnratio=0.6, bool checkOri=true);

    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    int SearchByProjection(ORBFrame &F, const std::vector<ORBMapPoint*> &vpMapPoints, const float th=3);

    // Project MapPoints tracked in last frame into the current frame and search matches.
    // Used to track from previous frame (Tracking)
    int SearchByProjection(ORBFrame &CurrentFrame, const ORBFrame &LastFrame, const float th, const bool bMono);

    // Project MapPoints seen in ORBKeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    int SearchByProjection(ORBFrame &CurrentFrame, ORBKeyFrame* pKF, const std::set<ORBMapPoint*> &sAlreadyFound, const float th, const int ORBdist);

    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
     int SearchByProjection(ORBKeyFrame* pKF, cv::Mat Scw, const std::vector<ORBMapPoint*> &vpPoints, std::vector<ORBMapPoint*> &vpMatched, int th);

    // Search matches between MapPoints in a ORBKeyFrame and ORB in a ORBFrame.
    // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
    // Used in Relocalisation and Loop Detection
    int SearchByBoW(ORBKeyFrame *pKF, ORBFrame &F, std::vector<ORBMapPoint*> &vpMapPointMatches);
    int SearchByBoW(ORBKeyFrame *pKF1, ORBKeyFrame* pKF2, std::vector<ORBMapPoint*> &vpMatches12);

    // Matching for the Map Initialization (only used in the monocular case)
    int SearchForInitialization(ORBFrame &F1, ORBFrame &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

    // Matching to triangulate new MapPoints. Check Epipolar Constraint.
    int SearchForTriangulation(ORBKeyFrame *pKF1, ORBKeyFrame* pKF2, cv::Mat F12,
                               std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);

    // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
    // In the stereo and RGB-D case, s12=1
    int SearchBySim3(ORBKeyFrame* pKF1, ORBKeyFrame* pKF2, std::vector<ORBMapPoint *> &vpMatches12, const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th);

    // Project MapPoints into ORBKeyFrame and search for duplicated MapPoints.
    int Fuse(ORBKeyFrame* pKF, const vector<ORBMapPoint *> &vpMapPoints, const float th=3.0);

    // Project MapPoints into ORBKeyFrame using a given Sim3 and search for duplicated MapPoints.
    int Fuse(ORBKeyFrame* pKF, cv::Mat Scw, const std::vector<ORBMapPoint*> &vpPoints, float th, vector<ORBMapPoint *> &vpReplacePoint);

public:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;


protected:

    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const ORBKeyFrame *pKF);

    float RadiusByViewingCos(const float &viewCos);

    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    float mfNNratio;
    bool mbCheckOrientation;
};

}

#endif // _ORBMATCHER_H_
