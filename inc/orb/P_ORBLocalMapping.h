

#ifndef _ORBLOCALMAPPING_H_
#define _ORBLOCALMAPPING_H_

#include "P_ORBKeyFrame.h"
#include "P_ORBLoopClosing.h"
#include "P_ORBTracking.h"
#include "P_ORBKeyFrameDatabase.h"

#include <mutex>


namespace Position
{

    class Tracking;
    class LoopClosing;

    class LocalMapping
    {
    public:
        LocalMapping(IMap* pMap, const float bMonocular);

        void SetLoopCloser(LoopClosing* pLoopCloser);

        void SetTracker(Tracking* pTracker);

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

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        IMap* mpMap;

        LoopClosing* mpLoopCloser;
        Tracking* mpTracker;

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
