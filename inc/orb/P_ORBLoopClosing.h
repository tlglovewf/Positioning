

#ifndef _ORBLOOPCLOSING_H_
#define _ORBLOOPCLOSING_H_

#include "P_ORBKeyFrame.h"
#include "P_ORBLocalMapping.h"
#include "P_ORBVocabulary.h"
#include "P_ORBTracking.h"

#include "P_ORBKeyFrameDatabase.h"

#include <thread>
#include <mutex>
#include <unistd.h>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace Position
{

    class Tracking;
    class LocalMapping;
    class KeyFrameDatabase;


    class LoopClosing
    {
    public:

        typedef pair<set<ORBKeyFrame*>,int> ConsistentGroup;    
        typedef map<ORBKeyFrame*,g2o::Sim3,std::less<ORBKeyFrame*>,
            Eigen::aligned_allocator<std::pair<ORBKeyFrame *const, g2o::Sim3> > > KeyFrameAndPose;

    public:

        LoopClosing(IMap* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc,const bool bFixScale);

        void SetTracker(Tracking* pTracker);

        void SetLocalMapper(LocalMapping* pLocalMapper);

        // Main function
        void Run();

        void InsertKeyFrame(ORBKeyFrame *pKF);

        void RequestReset();

        // This function will run in a separate thread
        void RunGlobalBundleAdjustment(unsigned long nLoopKF);

        bool isRunningGBA(){
            unique_lock<std::mutex> lock(mMutexGBA);
            return mbRunningGBA;
        }
        bool isFinishedGBA(){
            unique_lock<std::mutex> lock(mMutexGBA);
            return mbFinishedGBA;
        }   

        void RequestFinish();

        bool isFinished();

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:

        bool CheckNewKeyFrames();

        bool DetectLoop();

        bool ComputeSim3();

        void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

        void CorrectLoop();

        void ResetIfRequested();
        bool mbResetRequested;
        std::mutex mMutexReset;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        IMap* mpMap;
        Tracking* mpTracker;

        KeyFrameDatabase* mpKeyFrameDB;
        ORBVocabulary* mpORBVocabulary;

        LocalMapping *mpLocalMapper;

        std::list<ORBKeyFrame*> mlpLoopKeyFrameQueue;

        std::mutex mMutexLoopQueue;

        // Loop detector parameters
        float mnCovisibilityConsistencyTh;

        // Loop detector variables
        ORBKeyFrame* mpCurrentKF;
        ORBKeyFrame* mpMatchedKF;
        std::vector<ConsistentGroup> mvConsistentGroups;
        std::vector<ORBKeyFrame*> mvpEnoughConsistentCandidates;
        std::vector<ORBKeyFrame*> mvpCurrentConnectedKFs;
        MapPtVector mvpCurrentMatchedPoints;
        MapPtVector mvpLoopMapPoints;
        cv::Mat mScw;
        g2o::Sim3 mg2oScw;

        u64 mLastLoopKFid;

        // Variables related to Global Bundle Adjustment
        bool mbRunningGBA;
        bool mbFinishedGBA;
        bool mbStopGBA;
        std::mutex mMutexGBA;
        std::thread* mpThreadGBA;

        // Fix scale in the stereo/RGB-D case
        bool mbFixScale;


        int mnFullBAIdx;
    };

}

#endif // _ORBLOOPCLOSING_H_
