#ifndef _ORBKEYFRAME_H_
#define _ORBKEYFRAME_H_

#include "P_ORBMapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "P_Interface.h"
#include "P_ORBVocabulary.h"

namespace Position
{
    class ORBMapPoint;
    class ORBFrame;
    class KeyFrameDatabase;

    class ORBKeyFrame : public IKeyFrame
    {
    public:
        ORBKeyFrame(ORBFrame &F, IMap* pMap, KeyFrameDatabase* pKFDB);


        //重载类型转换
        virtual operator IFrame*()const 
        {
            assert(NULL);
        }

        //设置位置(R|t 矩阵)
        virtual void setPose(const cv::Mat &Tcw);
        //获取位置
        virtual cv::Mat getPose();
        //序号
        virtual u64 index()const {return mnId;}

        // Set/check bad flag
        virtual void setBadFlag();
        virtual bool isBad();

        //帧目标
        virtual TargetVector& getTargets() 
        {
            assert(NULL);
        }

        //更新下一帧
        virtual void updateNext(IKeyFrame *next) 
        {
            assert(NULL);
        }
        //更新上一帧
        virtual void updatePrev(IKeyFrame *pre) 
        {
            assert(NULL);
        }
        //获取到下一帧
        virtual IKeyFrame* getNext();
        //获取上一帧
        virtual IKeyFrame* getPrev();

        virtual void addMapPoint(IMapPoint* pMP, int idx);

        //是否已有对应地图点
        virtual bool hasMapPoint(int index);
       
        //移除地图点
        virtual void rmMapPoint(IMapPoint *pMP);
        virtual void rmMapPoint(int idx);

        //获取地图点
        virtual const MapPtVector& getPoints();

         //获取旋转 平移分量
        virtual Mat getRotation();
        virtual Mat getTranslation();

        cv::Mat GetPoseInverse();
        cv::Mat GetCameraCenter();

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
        const std::set<ORBKeyFrame*>& GetChilds();
        ORBKeyFrame* GetParent();
        bool hasChild(ORBKeyFrame* pKF);

        // Loop Edges
        void AddLoopEdge(ORBKeyFrame* pKF);
        std::set<ORBKeyFrame*> GetLoopEdges();

        // ORBMapPoint observation functions
        
        void ReplaceMapPointMatch(const size_t &idx, ORBMapPoint* pMP);
        MapPtSet    GetMapPoints();
        int TrackedMapPoints(const int &minObs);
        ORBMapPoint* GetMapPoint(const size_t &idx);

        // KeyPoint functions
        SzVector GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;

        // Image
        bool IsInImage(const float &x, const float &y) const;

        // Enable/Disable bad flag changes
        void SetNotErase();
        void SetErase();

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

        static u64 nNextId;
        u64 mnId;
        const u64 mnFrameId;

        const double mTimeStamp;

        // Grid (to speed up feature matching)
        const int mnGridCols;
        const int mnGridRows;
        const float mfGridElementWidthInv;
        const float mfGridElementHeightInv;

        // Variables used by the tracking
        u64 mnTrackReferenceForFrame;
        u64 mnFuseTargetForKF;

        // Variables used by the local mapping
        u64 mnBALocalForKF;
        u64 mnBAFixedForKF;

        // Variables used by the keyframe database
        u64 mnLoopQuery;
        int mnLoopWords;
        float mLoopScore;
        u64 mnRelocQuery;
        int mnRelocWords;
        float mRelocScore;

        // Variables used by loop closing
        cv::Mat mTcwGBA;
        cv::Mat mTcwBefGBA;
        u64 mnBAGlobalForKF;

        // Calibration parameters
        const float fx, fy, cx, cy, invfx, invfy;

        // Number of KeyPoints
        const int N;

        // KeyPoints, stereo coordinate and descriptors (all associated by an index)
        const KeyPtVector mvKeys;
        const KeyPtVector mvKeysUn;
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
        const FloatVector mvScaleFactors;
        const FloatVector mvLevelSigma2;
        const FloatVector mvInvLevelSigma2;

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
        MapPtVector mvpMapPoints;

        // BoW
        KeyFrameDatabase* mpKeyFrameDB;
        ORBVocabulary* mpORBvocabulary;

        // Grid over the image to speed up feature matching
        std::vector< std::vector < SzVector > > mGrid;

        KeyFrameMap mConnectedKeyFrameWeights; //帧,共视点数(权值)
        std::vector<ORBKeyFrame*> mvpOrderedConnectedKeyFrames;//根据上权值排序的共视帧
        std::vector<int> mvOrderedWeights;//排序权值

        // Spanning Tree and Loop Edges
        bool mbFirstConnection;
        ORBKeyFrame* mpParent;
        std::set<ORBKeyFrame*> mspChildrens;
        std::set<ORBKeyFrame*> mspLoopEdges;

        IKeyFrame *mpPrev;
        IKeyFrame *mpNext;

        // Bad flags
        bool mbNotErase;
        bool mbToBeErased;
        bool mbBad;    

        IMap* mpMap;

        std::mutex mMutexPose;
        std::mutex mMutexConnections;
        std::mutex mMutexFeatures;
    };
#define ORBKEYFRAME(P)  dynamic_cast<ORBKeyFrame*>(P)
} 

#endif // _ORBKEYFRAME_H_
