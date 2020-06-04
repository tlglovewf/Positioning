/**
 *   P_MultiVisionTrajProcesser.h
 *   
 *   add by tu li gen   2020.3.20
 * 
 */
#include "P_TrajProcesser.h"

namespace Position
{

    //多视图场景跟踪
    class PMultiVisionTrajProcesser : public PTrajProcesser
    {
    public:
         //构造
         PMultiVisionTrajProcesser(const std::shared_ptr<IConfig> &pcfg,
                                   const CameraParam &cam);

         //跟踪
        virtual cv::Mat track( FrameData *data);


         //处理
        virtual bool process(const FrameDataPtrVector &framedatas);
    protected:
        //关键帧创建
        virtual bool needCreateNewKeyFrame()
        {
            return true;
        }

        //创建新关键帧
        virtual IKeyFrame* createNewKeyFrame();

    protected:
        std::shared_ptr<IFeature>            mpFeature;
        std::unique_ptr<IFeatureMatcher>     mpFeatureMatcher;
        std::unique_ptr<IPoseSolver>         mpEst;
        std::unique_ptr<IOptimizer>          mpOptimizer;
        CameraParam                          mCam;
        int                                  mFtSearchRadius;
    };
    
} // namespace Position
