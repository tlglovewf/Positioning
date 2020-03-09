
/**
 *   P_Map.h
 *   
 *   add by tu li gen   2020.2.19
 * 
 */

#ifndef __PMAP_H_H_
#define __PMAP_H_H_
#include "P_Interface.h"
#include "P_Types.h"
#include "P_MapPoint.h"
#include "P_Frame.h"
namespace Position
{
    //地图对象
    class PMap : public IMap
    {
    public:
        PMap():mpCurrent(NULL),mMaxFmId(0),mnFrameCnt(0),mnMapPtCnt(0)
        {
        }
        ~PMap()
        {
            clear();
        }
        //创建关键帧
        IKeyFrame* createKeyFrame(IFrame *frame)
        {
            IKeyFrame *pF = new PKeyFrame(frame,mpCurrent,this);
            addKeyFrame(pF);
            return pF;
        }
        //创建地图点
        IMapPoint* createMapPoint(const cv::Mat &pose)
        {
            IMapPoint *pPt = new PMapPoint(pose,this,mnMapPtCnt++);
            addMapPoint(pPt);
            return pPt;
        }
        IMapPoint* createMapPoint(const cv::Point3f &pose)
        {
            IMapPoint *pPt = new PMapPoint(pose,this,mnMapPtCnt++);
            addMapPoint(pPt);
            return pPt;
        }

        //加入/移除关键帧
        void addKeyFrame(IKeyFrame *pKF)
        {
            assert(NULL != pKF);
            if(NULL != mpCurrent)mpCurrent->updateNext(pKF);
            mpCurrent = pKF;
            mMapFms.insert(pKF);
            if( pKF->index() > mMaxFmId)
                mMaxFmId = pKF->index();
        }
        void rmKeyFrame(IKeyFrame *pKF)
        {
            if(NULL != pKF)
            { //重新建立链接
                IKeyFrame *pre = pKF->getPre();
                IKeyFrame *nxt = pKF->getNext();
                if(pre)pre->updateNext(nxt);
                if(nxt)nxt->updatePre(pre);
                mMapFms.erase(pKF);
                pKF->release();
            }
        }
        //加入/移除地图点
        void addMapPoint(IMapPoint *pMp)
        {
            assert(NULL != pMp);
            mMapPts.insert(pMp);
        }
        void rmMapPoint(IMapPoint *pMp)
        {
            if(NULL != pMp)
            {
                mMapPts.erase(pMp);
                pMp->release();
            }
        }
        //清空
        void clear();

        //获取所有地图点
        MapPtVector getAllMapPts()const
        {
            return MapPtVector(mMapPts.begin(),mMapPts.end());
        }

        //获取所有帧
        KeyFrameVector getAllFrames()const
        {
            return KeyFrameVector(mMapFms.begin(),mMapFms.end());
        }

        //最大帧号
        virtual u64 getMaxKFid()const 
        {
            return mMaxFmId;
        }
         //获取帧，点计数
        virtual u64 frameCount() 
        {
            return mnFrameCnt++;
        }
        virtual u64 mapptCount() 
        {
            return mnMapPtCnt++;
        }
    protected:
        MapPtSet    mMapPts;
        KeyFmSet    mMapFms;
        IKeyFrame  *mpCurrent;
        u64         mMaxFmId;

        u64         mnFrameCnt;
        u64         mnMapPtCnt;
    private:
        DISABLEDCP(PMap)
    };


   
}

#endif