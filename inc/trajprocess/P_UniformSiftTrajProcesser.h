#ifndef __UNIFORMSIFTTRAJ_H_H_
#define __UNIFORMSIFTTRAJ_H_H_

#include "P_TrajProcesser.h"
#include "P_Task.h"
namespace Position
{
    //多视图场景跟踪
    class PUniformSiftTrajProcesser : public PTrajProcesser
    {
    public:
        //! 构造
        PUniformSiftTrajProcesser();

        //! 跟踪
        virtual cv::Mat track(FrameData *data);

        //! 处理
        virtual bool process(const FrameDataPtrVector &framedatas);

    protected:
        //关键帧创建
        virtual bool needCreateNewKeyFrame()
        {
            return true;
        }

        //创建新关键帧
        virtual IKeyFrame *createNewKeyFrame();

    protected:
        std::shared_ptr<IFeature> mpFeature;
        std::shared_ptr<IFeatureMatcher> mpFeatureMatcher;
        std::shared_ptr<IPoseSolver> mpEst;
        std::shared_ptr<IOptimizer> mpOptimizer;
        CameraParam mCam;
        int mFtSearchRadius;

        FeatureTask                 mFeatureTask;
        MatcherTask                 mMatcherTask;
        PoseTask                    mPoseTask;

    };
    DECLAREIFACTORY(ITrajProcesser, PUniformSiftTrajProcesser, SiftTraj)
} // namespace Position

#endif