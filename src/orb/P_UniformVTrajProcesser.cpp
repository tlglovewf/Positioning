#include "P_UniformVTrajProcesser.h"
#include "P_ORBMap.h"
#include "P_ORBTracking.h"
#include "P_ORBLocalMapping.h"
#include "P_ORBLoopClosing.h"
#include "P_ORBKeyFrameDatabase.h"
#include "P_Writer.h"
#include <unistd.h>

namespace Position
{
    //构造函数
    UniformVTrajProcesser::UniformVTrajProcesser(const std::shared_ptr<IConfig> &pcfg, const std::shared_ptr<IData> &pdata):PTrajProcesser(std::make_shared<ORBMap>()),mbReset(false)
    {

        std::string vocpath = GETCFGVALUE(pcfg,VocPath,string);

        mpVocabulary = std::make_shared<ORBVocabulary>();
        PROMT_S("Begin to load vocabulary!")
        // bool bVocLoad = mpVocabulary->loadFromTextFile(vocpath);
        // if(!bVocLoad)
        // {
        //     PROMT_S("vocabulary path error~!");
        //     PROMT_V("Failed to open at: ",vocpath.c_str());
        //     exit(-1);
        // }
        PROMT_S("Vocabulary loaded !!!");

        mpKeyFrameDatabase = std::make_shared<ORBKeyFrameDatabase>(mpVocabulary);

        mpTracker = std::make_shared<ORBTracking>(mpVocabulary,mpMap,mpKeyFrameDatabase,pcfg,pdata->getCamera());

        mpLocalMapper = std::make_shared<ORBLocalMapping>(mpMap, true);
        mptLocalMapping = std::unique_ptr<thread>( new thread(&Position::ORBLocalMapping::Run,mpLocalMapper.get()));

        mpLoopCloser = std::make_shared<ORBLoopClosing>(mpMap, mpKeyFrameDatabase, mpVocabulary, false);
        mptLoopClosing = std::unique_ptr<thread>(new thread(&Position::ORBLoopClosing::Run, mpLoopCloser.get()));

        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);

        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
    }

    UniformVTrajProcesser::~UniformVTrajProcesser()
    {
        // over();
    }

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
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = true;
        }

        return mpTracker->track(data);
    }

    //重置
    void UniformVTrajProcesser::reset()
    {
        mbReset = true;
    }

    //结束
    void UniformVTrajProcesser::over()
    {
        mpLocalMapper->RequestFinish();
        mpLoopCloser->RequestFinish();

        while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
        {
            usleep(5000);
        }
    }
}