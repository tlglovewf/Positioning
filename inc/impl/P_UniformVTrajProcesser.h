/**
 *   P_UniformVTrajProcesser.h
 *   
 *   add by tu li gen   2020.3.3
 * 
 */
#ifndef __PUNIFORMVTRAJPROCESSER_H_H_
#define __PUNIFORMVTRAJPROCESSER_H_H_
#include "P_TrajProcesser.h"

namespace Position
{
    //匀速跟踪
    class UniformVTrajProcesser : public PTrajProcesser
    {
    public:
        //构造
        UniformVTrajProcesser(const std::shared_ptr<IMap> &pmap):PTrajProcesser(pmap){
            mVelocity = cv::Mat::eye(4,4,MATCVTYPE);
        }

         //跟踪
        virtual cv::Mat track(const FrameData &data);

    protected:

        //是否能创建新帧
        virtual bool needCreateNewKeyFrame();

        //匀速运动跟踪
        void trackWithMotionModel();

    private:

        cv::Mat     mVelocity;
    };
}

#endif