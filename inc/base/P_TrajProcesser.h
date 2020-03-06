/**
 *   P_TrajProcesser.h
 *   
 *   add by tu li gen   2020.3.3
 * 
 */
#ifndef __PTRAJPROCESSER_H_H_
#define __PTRAJPROCESSER_H_H_
#include "P_Interface.h"

namespace Position
{

    //跟踪对象
    class PTrajProcesser : public ITrajProcesser
    {
    public:
        //构造
        PTrajProcesser( const std::shared_ptr<IMap> &pmap):
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

         //处理
        virtual bool process(const FrameDataVector &framedatas) 
        {
            if(framedatas.size() < 2)
            {
                return false;
            }
            else
            {
                for(size_t i = 0; i < framedatas.size(); ++i)
                {
                    track(framedatas[i]);
                }
                return true;
            }
        }

    protected:

        //是否能创建新帧
        virtual bool needCreateNewKeyFrame()
        {
            return true;
        }

        //创建新帧
        virtual void createNewKeyFrame()
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
}

#endif