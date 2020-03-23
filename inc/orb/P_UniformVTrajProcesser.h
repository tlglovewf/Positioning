/**
 *   P_UniformVTrajProcesser.h
 *   
 *   add by tu li gen   2020.3.3
 * 
 */
#ifndef __PUNIFORMVTRAJPROCESSER_H_H_
#define __PUNIFORMVTRAJPROCESSER_H_H_
#include "P_TrajProcesser.h"
#include "P_ORBVocabulary.h"
#include <thread>

namespace Position
{
    class ORBTracking;
    class ORBLocalMapping;
    class ORBLoopClosing;
    class ORBKeyFrameDatabase;

    //匀速跟踪
    class PUniformVTrajProcesser : public PTrajProcesser
    {
    public:
        //构造
        PUniformVTrajProcesser(const std::shared_ptr<IConfig> &pcfg, const std::shared_ptr<IData> &pdata);
        ~PUniformVTrajProcesser();
         //跟踪
        virtual cv::Mat track(const FrameData &data);

        //重置
        virtual void reset();

        //结束
        virtual void over();
    protected:
        //等待处理结束
        virtual void waitForProc();

    private:
       std::unique_ptr<std::thread>         mptLocalMapping;
       std::unique_ptr<std::thread>         mptLoopClosing;
  
       std::shared_ptr<ORBTracking>         mpTracker;
       std::shared_ptr<ORBLocalMapping>     mpLocalMapper;
       std::shared_ptr<ORBLoopClosing>      mpLoopCloser;
       std::shared_ptr<ORBVocabulary>       mpVocabulary;
       std::shared_ptr<ORBKeyFrameDatabase> mpKeyFrameDatabase;
       
       bool                 mbReset;
    };
}

#endif