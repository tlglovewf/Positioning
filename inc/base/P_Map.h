
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
        PMap(){}
        ~PMap()
        {
            clear();
        }
        //创建关键帧
        IKeyFrame* createKeyFrame(IFrame *frame)
        {
            IKeyFrame *pF = new PKeyFrame(frame,this);
            addKeyFrame(pF);
            return pF;
        }
        //创建地图点
        IMapPoint* createMapPoint(const cv::Mat &pose)
        {
            IMapPoint *pPt = new PMapPoint(pose,this);
            addMapPoint(pPt);
            return pPt;
        }
        IMapPoint* createMapPoint(const cv::Point3f &pose)
        {
            IMapPoint *pPt = new PMapPoint(pose,this);
            addMapPoint(pPt);
            return pPt;
        }

        //加入/移除关键帧
        void addKeyFrame(IKeyFrame *pKF)
        {
            if(NULL != pKF)
            {
                mMapFms.insert(pKF);
            }
        }
        void rmKeyFrame(IKeyFrame *pKF)
        {
            if(NULL != pKF)
            {
                mMapFms.erase(pKF);
            }
        }
        //加入/移除地图点
        void addMapPoint(IMapPoint *pMp)
        {
            if(NULL != pMp)
            {
                mMapPts.insert(pMp);
            }
        }
        void rmMapPoint(IMapPoint *pMp)
        {
            if(NULL != pMp)
            {
                mMapPts.erase(pMp);
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
    protected:
        MapPtSet    mMapPts;
        KeyFmSet    mMapFms;

    private:
        DISABLEDCP(PMap)
    };


   
}

#endif