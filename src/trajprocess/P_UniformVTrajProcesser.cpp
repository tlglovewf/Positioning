#include "P_UniformVTrajProcesser.h"
#include "P_ORBMap.h"
#include "P_ORBTracking.h"
#include "P_ORBLocalMapping.h"
#include "P_ORBLoopClosing.h"
#include "P_ORBKeyFrameDatabase.h"
#include "P_IOHelper.h"
#include "P_Utils.h"
#include "P_Checker.h"
#include "P_ORBVocabulary.h"
#include <unistd.h>

namespace Position
{
    //构造函数
    PUniformVTrajProcesser::PUniformVTrajProcesser():PTrajProcesser(std::make_shared<ORBMap>()),mbReset(false)
    {
        mpVocabulary = std::make_shared<ORBVocabulary>();
        const string path = "ORBvoc.bin";
        if(!PATHCHECK(path))
        {
            LOG_CRIT_F("%s Vocabulary File Not Found.Please Check The File At The Program Dir.",path.c_str());
            exit(-1);
        }
        LOG_INFO("Begin to load vocabulary!")
        bool bVocLoad = mpVocabulary->loadFromBinaryFile(path);
        if(!bVocLoad)
        {
            LOG_CRIT("Vocabulary Format Error .");  
            exit(-1);
        }
        LOG_INFO("Vocabulary Data Loading Finished.");

        mpKeyFrameDatabase = std::make_shared<ORBKeyFrameDatabase>(mpVocabulary);

        mpTracker = std::make_shared<ORBTracking>(mpVocabulary,mpMap,mpKeyFrameDatabase,GETGLOBALCONFIG(),GETGLOBALCONFIG()->getCamera());

        mpLocalMapper = std::make_shared<ORBLocalMapping>(mpMap);
        mptLocalMapping = std::unique_ptr<thread>( new thread(&Position::ORBLocalMapping::Run,mpLocalMapper.get()));
   
        //mpLoopCloser = std::make_shared<ORBLoopClosing>(mpMap, mpKeyFrameDatabase, mpVocabulary, false);
        //mptLoopClosing = std::unique_ptr<thread>(new thread(&Position::ORBLoopClosing::Run, mpLoopCloser.get()));

        mpTracker->SetLocalMapper(mpLocalMapper);
        //mpTracker->SetLoopClosing(mpLoopCloser);

        mpLocalMapper->SetTracker(mpTracker);
        //mpLocalMapper->SetLoopCloser(mpLoopCloser);

        //mpLoopCloser->SetTracker(mpTracker);
        //mpLoopCloser->SetLocalMapper(mpLocalMapper);
    }

    PUniformVTrajProcesser::~PUniformVTrajProcesser()
    {
       over();
    }

    //处理
    bool PUniformVTrajProcesser::process(const FrameDataPtrVector &framedatas) 
        {
            LOG_INFO("Use UniformV Process ...");
            if(framedatas.size() < 2)
            {
                return false;
            }
            else
            {
                for(size_t i = 0; i < framedatas.size(); ++i)
                {

                    track(framedatas[i]);

                    if(mpViewer)
                    {
                        waitKey(1);
                    }
                }
                wait();
                return true;
            }
        }

     //跟踪
    cv::Mat PUniformVTrajProcesser::track(FrameData *data)
    {
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
        LOG_INFO_F("Process :%s",data->_name.c_str());
        return mpTracker->track(data);
    }

    //等待
    void PUniformVTrajProcesser::wait()
    {
        while(!mpLocalMapper->AcceptKeyFrames())
        {
            usleep(100);
        }

        PROMTD_S("FrameDatas have Processed over.");
    }

    //重置
    void PUniformVTrajProcesser::reset()
    {
        mbReset = true;
    }

    //结束
    void PUniformVTrajProcesser::over()
    {
        if(mptLocalMapping && mptLocalMapping->joinable())
        {
            mpLocalMapper->RequestFinish();
            mptLocalMapping->join();
            LOG_DEBUG("Local Mapping Thread Over.");
        }
        
        if(mptLoopClosing && mptLoopClosing->joinable())
        {
            mpLoopCloser->RequestFinish();
            mptLoopClosing->join();
            LOG_DEBUG("Closing Thread Over.");
        }
    }
}