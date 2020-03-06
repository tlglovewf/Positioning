#include "P_UniformVTrajProcesser.h"
namespace Position
{

    //匀速运动模型跟踪
    void UniformVTrajProcesser::trackWithMotionModel()
    {

    }

    //关键帧创建条件
    bool UniformVTrajProcesser::needCreateNewKeyFrame()
    {
        return true;
    }

     //跟踪
    cv::Mat UniformVTrajProcesser::track(const FrameData &data)
    {
        return cv::Mat();
    }
}