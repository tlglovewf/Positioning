

#ifndef _ORBOPTIMIZER_H_
#define _ORBOPTIMIZER_H_

#include "P_ORBMapPoint.h"
#include "P_ORBKeyFrame.h"
#include "P_ORBLoopClosing.h"
#include "P_ORBFrame.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace Position
{

    class Optimizer
    {
    public:
        void static BundleAdjustment(const KeyFrameVector &vpKFs, const MapPtVector &vpMP,
                                    int nIterations = 5, bool *pbStopFlag = NULL, const unsigned long nLoopKF = 0,
                                    const bool bRobust = true);
        void static GlobalBundleAdjustemnt(const std::shared_ptr<IMap> &pMap, int nIterations = 5, bool *pbStopFlag = NULL,
                                        const unsigned long nLoopKF = 0, const bool bRobust = true);
        void static LocalBundleAdjustment(ORBKeyFrame *pKF, bool *pbStopFlag, const std::shared_ptr<IMap> &pMap);
        int static PoseOptimization(ORBFrame *pFrame);

        // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
        void static OptimizeEssentialGraph(const std::shared_ptr<IMap> &pMap, ORBKeyFrame *pLoopKF, ORBKeyFrame *pCurKF,
                                        const ORBLoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                        const ORBLoopClosing::KeyFrameAndPose &CorrectedSim3,
                                        const map<ORBKeyFrame *, set<ORBKeyFrame *>> &LoopConnections,
                                        const bool &bFixScale);

        // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
        static int OptimizeSim3(ORBKeyFrame *pKF1, ORBKeyFrame *pKF2, MapPtVector &vpMatches1,
                                g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
    };

} 

#endif // _ORBOPTIMIZER_H_