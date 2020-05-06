#include "P_UniformVTrajProcesser.h"
#include "P_ORBMap.h"
#include "P_ORBTracking.h"
#include "P_ORBLocalMapping.h"
#include "P_ORBLoopClosing.h"
#include "P_ORBKeyFrameDatabase.h"
#include "P_Writer.h"
#include "P_Utils.h"
#include <unistd.h>

namespace Position
{
    //构造函数
    PUniformVTrajProcesser::PUniformVTrajProcesser(const std::shared_ptr<IConfig> &pcfg, const std::shared_ptr<IData> &pdata):PTrajProcesser(std::make_shared<ORBMap>()),mbReset(false)
    {

        std::string vocpath = GETCFGVALUE(pcfg,VocPath,string);

        mpVocabulary = std::make_shared<ORBVocabulary>();
        PROMT_S("Begin to load vocabulary!")
        Time_Interval time;
        time.start();
        bool bVocLoad = mpVocabulary->loadFromTextFile(vocpath);
        if(!bVocLoad)
        {
            PROMT_S("vocabulary path error~!");  
            PROMT_V("Failed to open at: ",vocpath.c_str());
            exit(-1);
        }
        time.prompt("Voc loaded cost:");

        mpKeyFrameDatabase = std::make_shared<ORBKeyFrameDatabase>(mpVocabulary);

        mpTracker = std::make_shared<ORBTracking>(mpVocabulary,mpMap,mpKeyFrameDatabase,pcfg,pdata->getCamera());

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
            if(framedatas.size() < 2)
            {
                return false;
            }
            else
            {
                for(size_t i = 0; i < framedatas.size(); ++i)
                {
                    //track(framedatas, i);
                    
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

    cv::Mat PUniformVTrajProcesser::track(const FrameDataPtrVector &framedatas, const int initnum)
    {
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
        return mpTracker->InitMode(framedatas, initnum);
    }

     //跟踪
    cv::Mat PUniformVTrajProcesser::track(FrameData *data)
    {
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
        return mpTracker->track(data);
    }

    //等待
    void PUniformVTrajProcesser::wait()
    {
        while(!mpLocalMapper->AcceptKeyFrames())
        {
            usleep(100);
        }

        PROMT_S("FrameDatas have Processed over.");
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
            cout << "local mapping thread over." << endl;
        }
        
        if(mptLoopClosing && mptLoopClosing->joinable())
        {
            mpLoopCloser->RequestFinish();
            mptLoopClosing->join();
            cout << "closing thread over." << endl;
        }
    }
}