/**
 *   P_TrajProcesser.h
 *   
 *   add by tu li gen   2020.3.3
 * 
 */
#ifndef __PTRAJPROCESSER_H_H_
#define __PTRAJPROCESSER_H_H_
#include "P_Interface.h"
#include "P_Map.h"
namespace Position
{

    //跟踪对象
    class PTrajProcesser : public ITrajProcesser
    {
    public:
        PTrajProcesser(const std::shared_ptr<IMap> &pMap):
        mpMap(pMap),mStatus(eTrackNoImage),mpCurrent(NULL),mpLast(NULL),mpCurrentKeyFm(NULL),mpLastKeyFm(NULL)
        {

        }

        //构造
        PTrajProcesser():
        mpMap(new PMap()),mStatus(eTrackNoImage),mpCurrent(NULL),
        mpLast(NULL),mpCurrentKeyFm(NULL),mpLastKeyFm(NULL)
        {

        }
        //获取地图
        virtual const std::shared_ptr<IMap>& getMap() 
        {
            return mpMap;
        }
        //跟踪
        virtual cv::Mat track(const FrameData &data)
        {
            assert(NULL);
            return cv::Mat();
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
            mStatus         = eTrackNoImage;
            mpCurrent       = NULL;
            mpLast          = NULL;
            mpCurrentKeyFm  = NULL;
            mpLastKeyFm     = NULL;

            mpMap->clear();
        }

        //结束
        virtual void over() 
        {
            assert(NULL);
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

                    if(mpViewer)
                    {
                        waitKey(1);
                    }
                }
                wait();
                return true;
            }
        }
         //设置可视接口
        virtual void setViewer(const std::shared_ptr<IViewer> &viewer) 
        {   
            mpViewer = viewer;
            mpViewer->setMap(mpMap);
        }

        //等待
        virtual void wait()
        {
            assert(NULL);
        }
    protected:

        std::shared_ptr<IMap>       mpMap;
        std::shared_ptr<IViewer>    mpViewer;

        eTrackStatus                mStatus;

        IFrame                     *mpCurrent;
        IFrame                     *mpLast;
    
        IKeyFrame                  *mpCurrentKeyFm;
        IKeyFrame                  *mpLastKeyFm;
    };
}

#endif