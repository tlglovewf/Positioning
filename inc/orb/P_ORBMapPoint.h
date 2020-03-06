

#ifndef _ORBMAPPOINT_H_
#define _ORBMAPPOINT_H_

#include"P_ORBKeyFrame.h"
#include"P_ORBFrame.h"
#include"P_ORBMap.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace Position
{

class ORBKeyFrame;
class Map;
class ORBFrame;


class ORBMapPoint
{
public:
    ORBMapPoint(const cv::Mat &Pos, ORBKeyFrame* pRefKF, Map* pMap);
    ORBMapPoint(const cv::Mat &Pos,  Map* pMap, ORBFrame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    ORBKeyFrame* GetReferenceKeyFrame();

    std::map<ORBKeyFrame*,size_t> GetObservations();
    int Observations();

    void AddObservation(ORBKeyFrame* pKF,size_t idx);
    void EraseObservation(ORBKeyFrame* pKF);

    int GetIndexInKeyFrame(ORBKeyFrame* pKF);
    bool IsInKeyFrame(ORBKeyFrame* pKF);

    void SetBadFlag();
    bool isBad();

    void Replace(ORBMapPoint* pMP);    
    ORBMapPoint* GetReplaced();

    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    void ComputeDistinctiveDescriptors();

    cv::Mat GetDescriptor();

    void UpdateNormalAndDepth();

    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();
    int PredictScale(const float &currentDist, ORBKeyFrame*pKF);
    int PredictScale(const float &currentDist, ORBFrame* pF);

public:
    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:    

     // Position in absolute coordinates
     cv::Mat mWorldPos;

     // Keyframes observing the point and associated index in keyframe
     std::map<ORBKeyFrame*,size_t> mObservations;

     // Mean viewing direction
     cv::Mat mNormalVector;

     // Best descriptor to fast matching
     cv::Mat mDescriptor;

     // Reference ORBKeyFrame
     ORBKeyFrame* mpRefKF;

     // Tracking counters
     int mnVisible;
     int mnFound;

     // Bad flag (we do not currently erase ORBMapPoint from memory)
     bool mbBad;
     ORBMapPoint* mpReplaced;

     // Scale invariance distances
     float mfMinDistance;
     float mfMaxDistance;

     Map* mpMap;

     std::mutex mMutexPos;
     std::mutex mMutexFeatures;
};

} 

#endif // _ORBMAPPOINT_H_
