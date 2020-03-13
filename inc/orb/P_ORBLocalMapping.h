

#ifndef _ORBLOCALMAPPING_H_
#define _ORBLOCALMAPPING_H_

#include "P_Interface.h"

namespace Position
{
    class ORBLoopClosing;
    class ORBTracking;
    class ORBKeyFrameDatabase;
    class ORBMapPoint;
    class ORBKeyFrame;

    class ORBLocalMapping
    {
    public:
        ORBLocalMapping(const std::shared_ptr<IMap>& pMap, const float bMonocular);

        void SetLoopCloser(const std::shared_ptr<ORBLoopClosing>& pLoopCloser);

        void SetTracker(const std::shared_ptr<ORBTracking>& pTracker);

        // Main function
        void Run();

        void InsertKeyFrame(ORBKeyFrame* pKF);

        // Thread Synch
        void RequestStop();
        void RequestReset();
        bool Stop();
        void Release();
        bool isStopped();
        bool stopRequested();
        bool AcceptKeyFrames();
        void SetAcceptKeyFrames(bool flag);
        bool SetNotStop(bool flag);

        void InterruptBA();

        void RequestFinish();
        bool isFinished();

        int KeyframesInQueue(){
            unique_lock<std::mutex> lock(mMutexNewKFs);
            return mlNewKeyFrames.size();
        }

    protected:

        bool CheckNewKeyFrames();
        void ProcessNewKeyFrame();
        void CreateNewMapPoints();

        void MapPointCulling();
        void SearchInNeighbors();

        void KeyFrameCulling();

        cv::Mat ComputeF12(IKeyFrame* pKF1, IKeyFrame* pKF2);

        cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

        bool mbMonocular;

        void ResetIfRequested();
        bool mbResetRequested;
        std::mutex mMutexReset;

        bool CheckRequestFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        std::shared_ptr<IMap> mpMap;

        std::shared_ptr<ORBLoopClosing> mpLoopCloser;
        std::shared_ptr<ORBTracking> mpTracker;

        std::list<ORBKeyFrame*> mlNewKeyFrames;

        ORBKeyFrame* mpCurrentKeyFrame;

        std::list<ORBMapPoint*> mlpRecentAddedMapPoints;

        std::mutex mMutexNewKFs;

        bool mbAbortBA;

        bool mbStopped;
        bool mbStopRequested;
        bool mbNotStop;
        std::mutex mMutexStop;

        bool mbAcceptKeyFrames;
        std::mutex mMutexAccept;
    };

} 

#endif // _ORBLOCALMAPPING_H_
