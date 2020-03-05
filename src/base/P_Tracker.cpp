#include "P_Tracker.h"
namespace Position
{

    //匀速运动模型跟踪
    void UniformSpeedTracker::trackWithMotionModel()
    {

    }

    //关键帧创建条件
    bool UniformSpeedTracker::needCreateNewKeyFrame()
    {
        return true;
    }

     //跟踪
    cv::Mat UniformSpeedTracker::track(const FrameData &data)
    {
        return cv::Mat();
    }
}