#include "P_UniformSiftTrajProcesser.h"
#include "P_Factory.h"
#include "P_IOHelper.h"
#include "P_Frame.h"

#include "P_FeatureExtend.h"
#include <thread>

#define SAVEMATCHIMG    0  //是否存储同名点匹配文件

namespace Position
{


#define MINPOINTSIZEFORINIT  50

     //构造
    PUniformSiftTrajProcesser::PUniformSiftTrajProcesser():
                               mCam(GETGLOBALCONFIG()->getCamera()),
                               mFeatureTask("SiftEx",mCam),
                               mMatcherTask("Knn"),
                               mPoseTask("ORBPoseSolver",mCam)
                   {

                        mpOptimizer      = std::shared_ptr<IOptimizer>(GETOPTIMIZER());
                         
                        mpOptimizer->setCamera(mCam);    
                   }

    bool PUniformSiftTrajProcesser::process(const FrameDataPtrVector &framedatas)
    {
        LOG_INFO("Use UniformSift Process ...");
        if(framedatas.size() < 2)
        {
            return false;
        }
        else
        {
            
            for(size_t i = 0; i < framedatas.size(); ++i)
            {
                track(framedatas[i]);
#if !USE_VIEW
                framedatas[i]->_img.release();
#endif
                if(mpViewer)
                {
                    waitKey(1);
                }
            }

            // global optimization
            Position::KeyFrameVector keyframes(mpMap->getAllFrames());
            Position::MapPtVector    mappts(mpMap->getAllMapPts());

            bool pBstop = false;
            LOG_DEBUG_F("Begin Global Opt:%d-%d",keyframes.size(),mappts.size());
            mpOptimizer->bundleAdjustment(keyframes,mappts,5, &pBstop);
            LOG_DEBUG("Global Opt Finished.");

            return true;
        }
    }

    //跟踪
    cv::Mat PUniformSiftTrajProcesser::track(FrameData *data)
    {
        LOG_INFO_F("Process:%s",data->_name.c_str());

        if(mStatus == eTrackNoImage)
        {
            mpLast = mFeatureTask.run(data);
            mpLastKeyFm = mpMap->createKeyFrame(mpLast);
            mpLastKeyFm->setPose(Mat::eye(4,4,MATCVTYPE));
            mStatus = eTrackNoReady;
        }
        else if(mStatus == eTrackNoReady)
        {
            mpCurrent = mFeatureTask.run(data);
            MatcherTask::Item item(mpLast,mpCurrent);
            if(mMatcherTask.run(item) <  MINPOINTSIZEFORINIT)
            {
                LOG_WARNING("Match Size Not Enough!!!");
                return Mat();
            }
            else
            {
                PoseResult rst = mPoseTask.run(item);
                if(rst._vpts.empty())
                {
                    mpCurrent->release();
                    return Mat();
                }
                else
                {
                    LOG_INFO("Initilized Successfully.");
                    
                    mpCurrentKeyFm = mpMap->createKeyFrame(mpCurrent);

                    const MatchVector &match = rst._match;

                    LOG_DEBUG_F("%d -- %d",match.size(),rst._vpts.size());
                    assert(match.size() == rst._vpts.size());

                    for(size_t i = 0; i < match.size();++i)
                    {
                        IMapPoint *pt = mpMap->createMapPoint(rst._vpts[i]);
                        mpLastKeyFm->addMapPoint(pt,match[i].queryIdx);
                        mpCurrentKeyFm->addMapPoint(pt,match[i].trainIdx);
                    }
                    Mat pose = cv::Mat::eye(4,4,MATCVTYPE);
                    rst._R.copyTo(pose.rowRange(0,3).colRange(0,3));
                    rst._t.copyTo(pose.rowRange(0,3).col(3));

                    mpCurrentKeyFm->setPose(pose);
                    mStatus = eTrackOk;
                }
            }
        }
        else
        {
            LOG_INFO("Track Other Frames!!!");
            waitKey(0);
        }
        return Mat();
    }
}