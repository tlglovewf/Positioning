#include "P_TrajProcesser.h"

namespace Position
{

    //多视图场景跟踪
    class PSfmVisonTrajProcesser : public PTrajProcesser
    {
    public:
        //构造
        PSfmVisonTrajProcesser();
        
        ~PSfmVisonTrajProcesser();
        //! 跟踪
        virtual cv::Mat track( FrameData *data);


        //! 处理
        virtual bool process(const FrameDataPtrVector &framedatas);

    protected:
        //! 关键帧创建
        virtual bool needCreateNewKeyFrame()
        {
            return true;
        }

        //! 创建新关键帧
        virtual IKeyFrame* createNewKeyFrame();

    protected:
        std::shared_ptr<IFeature>            mpFeature;
        std::shared_ptr<IFeatureMatcher>     mpFeatureMatcher;
        std::shared_ptr<IPoseSolver>         mpEst;
        std::shared_ptr<IOptimizer>          mpOptimizer;
        CameraParam                          mCam;
        int                                  mFtSearchRadius;
    };
    DECLAREIFACTORY(ITrajProcesser, PSfmVisonTrajProcesser,SfmTraj)
} // namespace Position