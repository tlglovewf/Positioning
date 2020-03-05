/**
 *   P_Tracker.h
 *   
 *   add by tu li gen   2020.3.3
 * 
 */
#ifndef __PTRACKER_H_H_
#define __PTRACKER_H_H_
#include "P_Interface.h"

namespace Position
{

    //跟踪对象
    class PTracker : public ITracker
    {
    public:
        //构造
        PTracker( const std::shared_ptr<IMap> &pmap):
        mpMap(pmap),mStatus(eTrackPrepare),mpCurrent(NULL),
        mpLast(NULL),mpCurrentKeyFm(NULL),mpLastKeyFm(NULL)
        {

        }
        //跟踪
        virtual cv::Mat track(const FrameData &data)
        {
            assert(NULL);
        }

        //状态
        virtual eTrackStatus status()const
        {
            return mStatus;
        }

         //当前帧
        virtual IKeyFrame* current()const
        {
            return mpCurrentKeyFm;
        }
        //上一帧
        virtual IKeyFrame* last()const 
        {
            return mpLastKeyFm;
        }

        //重置
        virtual void reset()
        {//重置 地图清空 状态复位
            mStatus         = eTrackPrepare;
            mpCurrent       = NULL;
            mpLast          = NULL;
            mpCurrentKeyFm  = NULL;
            mpLastKeyFm     = NULL;

            mpMap->clear();
        }

    protected:

        //是否能创建新帧
        virtual bool needCreateNewKeyFrame()
        {
            return true;
        }

        //创建新帧
        void createNewKeyFrame()
        {
            if(NULL != mpCurrent)
            {
                mpLastKeyFm = mpCurrentKeyFm;
                mpCurrentKeyFm = mpMap->createKeyFrame(mpCurrent);
                //更新帧间关系
                mpCurrentKeyFm->updatePre(mpLastKeyFm);
                mpLastKeyFm->updateNext(mpCurrentKeyFm);
            }
        }
    
    protected:

        std::shared_ptr<IMap> mpMap;

        eTrackStatus          mStatus;

        IFrame               *mpCurrent;
        IFrame               *mpLast;

        IKeyFrame            *mpCurrentKeyFm;
        IKeyFrame            *mpLastKeyFm;
    };

    //匀速跟踪
    class UniformSpeedTracker : public PTracker
    {
    public:
        //构造
        UniformSpeedTracker(const std::shared_ptr<IMap> &pmap):PTracker(pmap){
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