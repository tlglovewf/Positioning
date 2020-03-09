

#ifndef _ORBMAPPOINT_H_
#define _ORBMAPPOINT_H_

#include"P_ORBKeyFrame.h"
#include"P_ORBFrame.h"
#include"P_ORBMap.h"
#include "P_Interface.h"

namespace Position
{
    class ORBKeyFrame;
    class Map;
    class ORBFrame;

    class ORBMapPoint :public IMapPoint
    {
    public:
        ORBMapPoint(const cv::Mat &Pos, ORBKeyFrame* pRefKF, Map* pMap);
        //设置世界位姿
        virtual void setWorldPos(const cv::Mat &Pos);
        //获取位姿
        virtual cv::Mat getWorldPos();
        //返回向量
        virtual cv::Mat normal();
         //序号
        virtual u64 index()const 
        {
            return mnId;
        }
        //设置坏点
        virtual void setBadFlag();

        //是否为坏点
        virtual bool isBad();

        //观察点
        virtual int observations() ;

        //添加观察者 index(特征点序号)
        virtual void addObservation(IKeyFrame *frame,int index);

        //移除观察者
        virtual void rmObservation(IKeyFrame *pKF);

        //获取观察帧列表
        virtual  KeyFrameMap getObservations();

        //是否在帧中
        virtual bool isInFrame(IKeyFrame *pKF);


        ORBKeyFrame* GetReferenceKeyFrame();



        int GetIndexInKeyFrame(ORBKeyFrame* pKF);
       

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
        KeyFrameMap mObservations;
        

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
