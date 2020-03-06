#ifndef _ORBKEYFRAME_H_
#define _ORBKEYFRAME_H_

#include "P_ORBMapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "P_ORBVocabulary.h"
#include "P_ORBFeature.h"
#include "P_ORBFrame.h"
#include "P_ORBKeyFrameDatabase.h"

#include <mutex>

namespace Position
{

class Map;
class ORBMapPoint;
class ORBFrame;
class KeyFrameDatabase;

class ORBKeyFrame
{
public:
    ORBKeyFrame(ORBFrame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    void SetPose(const cv::Mat &Tcw);
    cv::Mat GetPose();
    cv::Mat GetPoseInverse();
    cv::Mat GetCameraCenter();
    cv::Mat GetRotation();
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(ORBKeyFrame* pKF, const int &weight);
    void EraseConnection(ORBKeyFrame* pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<ORBKeyFrame *> GetConnectedKeyFrames();
    std::vector<ORBKeyFrame* > GetVectorCovisibleKeyFrames();
    std::vector<ORBKeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<ORBKeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(ORBKeyFrame* pKF);

    // Spanning tree functions
    void AddChild(ORBKeyFrame* pKF);
    void EraseChild(ORBKeyFrame* pKF);
    void ChangeParent(ORBKeyFrame* pKF);
    std::set<ORBKeyFrame*> GetChilds();
    ORBKeyFrame* GetParent();
    bool hasChild(ORBKeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(ORBKeyFrame* pKF);
    std::set<ORBKeyFrame*> GetLoopEdges();

    // ORBMapPoint observation functions
    void AddMapPoint(ORBMapPoint* pMP, const size_t &idx);
    void EraseMapPointMatch(const size_t &idx);
    void EraseMapPointMatch(ORBMapPoint* pMP);
    void ReplaceMapPointMatch(const size_t &idx, ORBMapPoint* pMP);
    std::set<ORBMapPoint*> GetMapPoints();
    std::vector<ORBMapPoint*> GetMapPointMatches();
    int TrackedMapPoints(const int &minObs);
    ORBMapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(ORBKeyFrame* pKF1, ORBKeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    static long unsigned int nNextId;
    long unsigned int mnId;
    const long unsigned int mnFrameId;

    const double mTimeStamp;

    // Grid (to speed up feature matching)
    const int mnGridCols;
    const int mnGridRows;
    const float mfGridElementWidthInv;
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    long unsigned int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    const float fx, fy, cx, cy, invfx, invfy;

    // Number of KeyPoints
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    const std::vector<cv::KeyPoint> mvKeys;
    const std::vector<cv::KeyPoint> mvKeysUn;
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    const int mnScaleLevels;
    const float mfScaleFactor;
    const float mfLogScaleFactor;
    const std::vector<float> mvScaleFactors;
    const std::vector<float> mvLevelSigma2;
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    const cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    cv::Mat Tcw;
    cv::Mat Twc;
    cv::Mat Ow;

    // MapPoints associated to keypoints
    std::vector<ORBMapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<ORBKeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<ORBKeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    ORBKeyFrame* mpParent;
    std::set<ORBKeyFrame*> mspChildrens;
    std::set<ORBKeyFrame*> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} 

#endif // _ORBKEYFRAME_H_
